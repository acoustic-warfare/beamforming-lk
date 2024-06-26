# Build
# docker build -t beamformer .

# docker run -v $(pwd):/usr/src/app -e DISPLAY=$DISPLAY -it --network=host beamformer bash
# docker run -v $(pwd):/usr/src/app -e DISPLAY=$DISPLAY -it --network=host -v /tmp/.X11-unix:/tmp/.X11-unix --user=$(id -u $USER) beamformer bash

FROM ubuntu:22.04

ENV TZ=Europe \
    DEBIAN_FRONTEND=noninteractive \
    DISPLAY=:0.0

RUN apt-get update -y

# Setting up build environment
RUN apt-get install -y \
    build-essential \
    cmake 

# Installing libraries
RUN apt-get install -y \
    libopencv-dev \
    libeigen3-dev \
    librtaudio-dev

RUN apt-get install -y \
    python3-yaml

# Setting up WARAPS

# WARAPS deps
RUN apt-get install -y \
    git \
    libboost-dev \
    libssl-dev \
    libpaho-mqtt-dev \
    libpaho-mqttpp-dev \
    nlohmann-json3-dev

RUN git clone https://github.com/acoustic-warfare/WARA-PS-MQTT-Agent.git 
RUN mkdir -p WARA-PS-MQTT-Agent/build
WORKDIR /WARA-PS-MQTT-Agent/build
RUN cmake .. && make install

# Create app directory
RUN mkdir -p /usr/src/app
# Change working dir to /usr/src/app
WORKDIR /usr/src/app


