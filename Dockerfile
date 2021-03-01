FROM debian:stretch

# Switch into our apps working directory
WORKDIR /home/martin/radio/345SecurityMQTT/src

RUN apt-get -q update && apt-get install -yq --no-install-recommends \
	librtlsdr-dev rtl-sdr libmosquittopp-dev \
	&& apt-get clean && rm -rf /var/lib/apt/lists/*

#YOU NEED TO BUILD THE PROJECT FIRST!
COPY ./src/345toMqtt 345toMqtt

#switch on systemd init system in container
ENV INITSYSTEM on

# Run our binary on container startup
CMD ./345toMqtt
