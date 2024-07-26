#
# Author: Irreq
#
# Dockerfile for building and running the project. Running container without root access
# is recomended since the container have read/write access to the directory.
# 
# 
# To build the container run the following command:
# 
# docker build -t beamformer .
# 
# To run the container
#
# docker run -v $(pwd):/usr/src/app -e DISPLAY=$DISPLAY -it --network=host -v /tmp/.X11-unix:/tmp/.X11-unix -e PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native -v ${XDG_RUNTIME_DIR}/pulse/native:${XDG_RUNTIME_DIR}/pulse/native -v /run/dbus:/run/dbus --device /dev/snd beamformer bash
#

FROM ubuntu:22.04 AS deps

ENV TZ=Europe \
    DEBIAN_FRONTEND=noninteractive \
    DISPLAY=:0.0

RUN apt-get update -y

# Setting up build environment
RUN apt-get install -y \
    build-essential \
    wget

# Setting up cmake
# renovate: datasource=docker depName=gcc versioning=docker
ARG GCC_VERSION=10

# renovate: datasource=github-releases depName=Kitware/CMake
ARG CMAKE_VERSION=3.23.0

RUN wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-Linux-x86_64.sh \
      -q -O /tmp/cmake-install.sh \
      && chmod u+x /tmp/cmake-install.sh \
      && mkdir /usr/bin/cmake \
      && /tmp/cmake-install.sh --skip-license --prefix=/usr/bin/cmake \
      && rm /tmp/cmake-install.sh

ENV PATH="/usr/bin/cmake/bin:${PATH}"

# Installing libraries
RUN apt-get install -y \
    libopencv-dev \
    libeigen3-dev \
    libasound-dev \
    libportaudiocpp0 \
    portaudio19-dev \
    python3-yaml \
    git \
    libboost-dev \
    libssl-dev \
    nlohmann-json3-dev \
    libgps-dev \
    libgps28 \
    alsa-utils \
    pulseaudio \
    libmp3lame-dev \
    libsndfile1-dev

RUN git clone https://github.com/eclipse/paho.mqtt.cpp
WORKDIR /paho.mqtt.cpp
RUN git submodule init \
    && git submodule update \
    && cmake -Bbuild -H. -DPAHO_WITH_MQTT_C=ON -DPAHO_WITH_SSL=ON \
    && cmake --build build/ --target install


RUN git clone https://github.com/Rookfighter/pso-cpp.git && \
    mkdir -p pso-cpp/build && \
    cd pso-cpp/build && \
    cmake .. && make install

FROM deps AS build

WORKDIR /
RUN git clone https://github.com/acoustic-warfare/WARA-PS-MQTT-Agent.git
WORKDIR /WARA-PS-MQTT-Agent
RUN cmake -S . -B build \
    && cmake --build build/ --target install

WORKDIR /
RUN git clone https://github.com/p-ranav/argparse.git
WORKDIR /argparse
RUN mkdir build && cd build && cmake .. && make -j4 && make -j4 install

RUN ldconfig

# Create app directory
RUN mkdir -p /usr/src/app

RUN useradd -rm -d /home/newuser -s /bin/bash -g root -G sudo -G audio -u 1000 newuser
USER newuser

# Set working directory
WORKDIR /usr/src/app

# Add configs for sound and start pulse audio
COPY daemon.conf /etc/pulse/daemon.conf
RUN pulseaudio --start