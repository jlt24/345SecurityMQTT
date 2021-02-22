#include "digitalDecoder.h"
#include "mqtt.h"
#include "mqtt_config.h"

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <csignal>


// Pulse checks seem to be about 60-70 minutes apart
#define RX_TIMEOUT_MIN      (90)

// Give each sensor 3 intervals before we flag a problem
#define SENSOR_TIMEOUT_MIN  (90*5)

#define SYNC_MASK    0xFFFF000000000000ul
#define SYNC_PATTERN 0xFFFE000000000000ul

// Don't send these messages more than once per minute unless there is a state change
#define RX_GOOD_MIN_SEC (60)
#define UPDATE_MIN_SEC (60)

#define BASE_TOPIC "security/sensors345/"
#define SENSOR_TOPIC BASE_TOPIC"sensor/"
#define KEYFOB_TOPIC BASE_TOPIC"keyfob/"
#define KEYPAD_TOPIC BASE_TOPIC"keypad/"

void DigitalDecoder::setRxGood(bool state)
{
    std::string topic(BASE_TOPIC);
    timeval now;

    topic += "rx_status";

    gettimeofday(&now, nullptr);

    if (rxGood != state || (now.tv_sec - lastRxGoodUpdateTime) > RX_GOOD_MIN_SEC)
    {
        mqtt.send(topic.c_str(), state ? "OK" : "FAILED");
    }

    // Reset watchdog either way
    alarm(RX_TIMEOUT_MIN*60);

    rxGood = state;
    lastRxGoodUpdateTime = now.tv_sec;
}

void DigitalDecoder::updateKeyfobState(uint32_t serial, uint64_t payload)
{
    if (payload == lastKeyfobPayload)
    {
        return;
    }

    std::ostringstream topic;
    topic << KEYFOB_TOPIC << serial << "/keypress";
    char c = ((payload & 0x000000F00000) >> 20);
    std::string key;
    if (c == 0x1)
    {
        key = "AWAY";
    }
    else if (c == 0x2)
    {
        key = "DISARM";
    }
    else if (c == 0x4)
    {
        key = "STAY";
    }
    else if (c == 0x8)
    {
        key = "AUX";
    }
    else
    {
        key = "UNK";
    }
    mqtt.send(topic.str().c_str(), key.c_str(), 1, false);

    lastKeyfobPayload = payload;
}

void DigitalDecoder::updateKeypadState(uint32_t serial, uint64_t payload)
{
    timeval now;
    gettimeofday(&now, nullptr);

    struct keypadState_t lastState;
    struct keypadState_t currentState;

    currentState.lastUpdateTime = now.tv_sec;
    currentState.hasLostSupervision = false;

    currentState.sequence = (payload & 0xF00000000000) >> 44;
    currentState.lowBat = payload & 0x000000020000;

    bool supervised = payload & 0x000000040000;
    if (supervised) return;

    auto found = keypadStatusMap.find(serial);
    if (found == keypadStatusMap.end())
    {
        lastState.sequence = 0xff;
        lastState.lowBat = !currentState.lowBat;
    }
    else
    {
        lastState = found->second;
    }

    if (currentState.sequence != lastState.sequence)
    {
        std::ostringstream topic;
        topic << KEYPAD_TOPIC << serial << "/keypress";
        char c = ((payload & 0x000000F00000) >> 20);

        std::string key;
        if (c == 0xA)
        {
            key = "*";
        }
        else if (c == 0xB)
        {
            key = "0";
        }
        else if (c == 0xC)
        {
            key = "#";
        }
        else if (c == 0xD)
        {
            key = "STAY";
        }
        else if (c == 0xE)
        {
            key = "AWAY";
        }
        else if (c == 0xF)
        {
            key = "FIRE";
        }
        else if (c == 0x0)
        {
            key = "POLICE";
        }
        else
        {
            key = (c + '0');
        }
        mqtt.send(topic.str().c_str(), key.c_str(), 1, false);

        if ((c >= 1 && c <= 0xC) && (currentState.lastUpdateTime <= (lastState.lastUpdateTime + 2)) && (lastState.phrase.length() < 10))
        {
            currentState.phrase = lastState.phrase + key;

            std::ostringstream phraseTopic;
            phraseTopic << KEYPAD_TOPIC << serial << "/keyphrase/" << currentState.phrase.length();
            mqtt.send(phraseTopic.str().c_str(), currentState.phrase.c_str(), 1, false);
        }
        else if (c == 0xB || (c >= 1 && c <= 9))
        {
            currentState.phrase = key;
        }

        keypadStatusMap[serial] = currentState;
    }
}

void DigitalDecoder::updateSensorState(uint32_t serial, uint64_t payload)
{
    timeval now;
    gettimeofday(&now, nullptr);

    struct sensorState_t lastState;
    struct sensorState_t currentState;

    currentState.lastUpdateTime = now.tv_sec;
    currentState.hasLostSupervision = false;

    currentState.loop1 = payload  & 0x000000800000;
    currentState.loop2 = payload  & 0x000000200000;
    currentState.loop3 = payload  & 0x000000100000;
    currentState.tamper = payload & 0x000000400000;
    currentState.lowBat = payload & 0x000000080000;

    // bool supervised = payload & 0x000000040000;
    // bool repeated = payload & 0x000000020000;

    //std::cout << "Payload:" << std::hex << payload << " Serial:" << std::dec << serial << std::boolalpha << " Loop1:" << currentState.loop1 << std::endl;

    auto found = sensorStatusMap.find(serial);
    if (found == sensorStatusMap.end())
    {
        // if there wasn't a state, make up a state that is opposite to our current state
        // so that we send everything.

        lastState.hasLostSupervision = !currentState.hasLostSupervision;
        lastState.loop1 = !currentState.loop1;
        lastState.loop2 = !currentState.loop2;
        lastState.loop3 = !currentState.loop3;
        lastState.tamper = !currentState.tamper;
        lastState.lowBat = !currentState.lowBat;
        lastState.lastUpdateTime = 0;
    }
    else
    {
        lastState = found->second;
    }

    // Since the sensor will frequently blast out the same signal many times, we only want to treat
    // the first detected signal as the supervisory signal.
    bool supervised = (payload & 0x000000040000) && ((currentState.lastUpdateTime - lastState.lastUpdateTime) > 2);

    if ((currentState.loop1 != lastState.loop1) || supervised)
    {
        std::ostringstream topic;
        topic << SENSOR_TOPIC << serial << "/loop1";
        mqtt.send(topic.str().c_str(), currentState.loop1 ? OPEN_SENSOR_MSG : CLOSED_SENSOR_MSG, supervised ? 0 : 1);
    }

    if ((currentState.loop2 != lastState.loop2) || supervised)
    {
        std::ostringstream topic;
        topic << SENSOR_TOPIC << serial << "/loop2";
        mqtt.send(topic.str().c_str(), currentState.loop2 ? OPEN_SENSOR_MSG : CLOSED_SENSOR_MSG, supervised ? 0 : 1);
    }

    if ((currentState.loop3 != lastState.loop3) || supervised)
    {
        std::ostringstream topic;
        topic << SENSOR_TOPIC << serial << "/loop3";
        mqtt.send(topic.str().c_str(), currentState.loop3 ? OPEN_SENSOR_MSG : CLOSED_SENSOR_MSG, supervised ? 0 : 1);
    }

    if ((currentState.tamper != lastState.tamper) || supervised)
    {
        std::ostringstream topic;
        topic << SENSOR_TOPIC << serial << "/tamper";
        mqtt.send(topic.str().c_str(), currentState.tamper ? TAMPER_MSG : UNTAMPERED_MSG, supervised ? 0 : 1);
    }

    if ((currentState.lowBat != lastState.lowBat) || supervised)
    {
        std::ostringstream topic;
        topic << SENSOR_TOPIC << serial << "/battery";
        mqtt.send(topic.str().c_str(), currentState.lowBat ? LOW_BAT_MSG : OK_BAT_MSG, supervised ? 0 : 1);
    }

    sensorStatusMap[serial] = currentState;
}

/* Checks all devices for last time updated */
void DigitalDecoder::checkForTimeouts()
{
    timeval now;
    std::ostringstream status;

    status << "TIMEOUT";
    gettimeofday(&now, nullptr);

    for(const auto &dd : sensorStatusMap)
    {
        if ((now.tv_sec - dd.second.lastUpdateTime) > SENSOR_TIMEOUT_MIN*60)
        {
            if (false == dd.second.hasLostSupervision)
            {
                std::ostringstream statusTopic;

                sensorStatusMap[dd.first].hasLostSupervision = true;
                statusTopic << BASE_TOPIC << dd.first << "/status";
                mqtt.send(statusTopic.str().c_str(), status.str().c_str());
            }
        }
    }
}

bool DigitalDecoder::isPayloadValid(uint64_t payload, uint64_t polynomial) const
{
    uint64_t sof = (payload & 0xF00000000000) >> 44;
    uint64_t ser = (payload & 0x0FFFFF000000) >> 24;
    uint64_t typ = (payload & 0x000000FF0000) >> 16;
    uint64_t crc = (payload & 0x00000000FFFF) >>  0;

    //
    // Check CRC
    //
    if (polynomial == 0)
    {
        if (sof == 0x2 /* 2gig smoke */ || sof == 0x3 /* 2gig panic */ || sof == 0x4 /* 2gig PIR */
            || sof == 0x7 /* 2gig flood/temp */ || sof == 0x9 /* 2gig glass break */
            || sof == 0xA /* 2gig door window */ || sof == 0xB /* 2gig carbon monoxide */
            || sof == 0xC /* 2gig Tilt */ || sof == 0xF /* Remote keyfob */) {
            // 2GIG brand
            polynomial = 0x18050;
        } else if (sof == 0x8) {
            // Honeywell Sensor
            printf("Honeywell Sensor");
            polynomial = 0x18005;
        } else if (sof == 0xD || sof == 0xE) {
            // Vivint
            printf("Vivint Sensor %ld", sof);
            polynomial = 0x18050; // Don't know if this is correct
        } else {
            // Something else?
            printf("Unknown Brand Sensor %ld", sof);
            polynomial = 0x18050;
        }
    }
    uint64_t sum = payload & (~SYNC_MASK);
    uint64_t current_divisor = polynomial << 31;

    while(current_divisor >= polynomial)
    {
        #ifdef __arm__
        if(__builtin_clzll(sum) == __builtin_clzll(current_divisor))
        #else
        if(__builtin_clzl(sum) == __builtin_clzl(current_divisor))
        #endif
        {
            sum ^= current_divisor;
        }
        current_divisor >>= 1;
    }

    return (sum == 0);
}

void DigitalDecoder::handlePayload(uint64_t payload)
{
    uint64_t ser = (payload & 0x0FFFFF000000) >> 24;
    uint64_t typ = (payload & 0x000000FF0000) >> 16;

    const bool validSensorPacket = isPayloadValid(payload);
    const bool validKeypadPacket = isPayloadValid(payload, 0x18050) && (typ & 0x01);
    const bool validKeyfobPacket = isPayloadValid(payload, 0x18050) && (typ & 0x02);

    //
    // Print Packet
    //
 #ifdef __arm__
    printf("%s Payload: %llX (Serial %llu/%llX, Status %llX)\n", (validSensorPacket | validKeypadPacket | validKeyfobPacket) ? "Valid" : "Invalid", payload, ser, ser, typ);
 #else
    printf("%s Payload: %lX (Serial %lu/%lX, Status %lX)\n", (validSensorPacket | validKeypadPacket | validKeyfobPacket) ? "Valid" : "Invalid", payload, ser, ser, typ);
 #endif

    packetCount++;
    if(!validSensorPacket && !validKeypadPacket && !validKeyfobPacket)
    {
        errorCount++;
        printf("%u/%u packets failed CRC", errorCount, packetCount);
        std::cout << std::endl;
    }

    //
    // Tell the world
    //
    if(validSensorPacket && keypadStatusMap.find(ser) == keypadStatusMap.end())
    {
        // We received a valid packet so the receiver must be working
        setRxGood(true);
        // Update the device
        updateSensorState(ser, payload);
    }
    else if (validKeypadPacket)
    {
        setRxGood(true);
        updateKeypadState(ser, payload);
    }
    else if (validKeyfobPacket)
    {
        setRxGood(true);
        updateKeyfobState(ser, payload);
    }
}



void DigitalDecoder::handleBit(bool value)
{
    static uint64_t payload = 0;

    payload <<= 1;
    payload |= (value ? 1 : 0);

//#ifdef __arm__
//    printf("Got bit: %d, payload is now %llX\n", value?1:0, payload);
//#else
//    printf("Got bit: %d, payload is now %lX\n", value?1:0, payload);
//#endif

    if((payload & SYNC_MASK) == SYNC_PATTERN)
    {
        handlePayload(payload);
        payload = 0;
    }
}

void DigitalDecoder::decodeBit(bool value)
{
    enum ManchesterState
    {
        LOW_PHASE_A,
        LOW_PHASE_B,
        HIGH_PHASE_A,
        HIGH_PHASE_B
    };

    static ManchesterState state = LOW_PHASE_A;

    switch(state)
    {
        case LOW_PHASE_A:
        {
            state = value ? HIGH_PHASE_B : LOW_PHASE_A;
            break;
        }
        case LOW_PHASE_B:
        {
            handleBit(false);
            state = value ? HIGH_PHASE_A : LOW_PHASE_A;
            break;
        }
        case HIGH_PHASE_A:
        {
            state = value ? HIGH_PHASE_A : LOW_PHASE_B;
            break;
        }
        case HIGH_PHASE_B:
        {
            handleBit(true);
            state = value ? HIGH_PHASE_A : LOW_PHASE_A;
            break;
        }
    }
}

void DigitalDecoder::handleData(char data)
{
    static const int samplesPerBit = 8;


    if(data != 0 && data != 1) return;

    const bool thisSample = (data == 1);

    if(thisSample == lastSample)
    {
        samplesSinceEdge++;

        //if(samplesSinceEdge < 100)
        //{
        //    printf("At %d for %u\n", thisSample?1:0, samplesSinceEdge);
        //}

        if((samplesSinceEdge % samplesPerBit) == (samplesPerBit/2))
        {
            // This Sample is a new bit
            decodeBit(thisSample);
        }
    }
    else
    {
        samplesSinceEdge = 1;
    }
    lastSample = thisSample;
}
