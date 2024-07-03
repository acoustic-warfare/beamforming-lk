#!/bin/bash

# Reset
Color_Off='\033[0m'       # Text Reset

# Regular Colors
Black='\033[0;30m'        # Black
Red='\033[0;31m'          # Red
Green='\033[0;32m'        # Green
Yellow='\033[0;33m'       # Yellow
Blue='\033[0;34m'         # Blue
Purple='\033[0;35m'       # Purple
Cyan='\033[0;36m'         # Cyan
White='\033[0;37m'        # White

# Checking that user do not run as root
if [ "$EUID" -eq 0 ]
  then echo "Please do not run as root"
  exit
fi

# Checking realtime-kernel
if [ -z "$(uname -a | grep PREEMPT_RT)" ]; then
    echo -e "$Red[WARNING]$Color_Off System does not have Realtime kernel enabled"
    echo "Please install one: https://ubuntu.com/pro"
    RT_ARGS=""
else
    RT_ARGS="--network=host --ulimit rtprio=99 --cpus=$(nproc) --cap-add=SYS_NICE"
fi

install_docker () {
   echo "Installing docker..."
   sudo apt install docker
}


# Checking docker installation
if ! [ -x "$(command -v docker)" ]; then
    echo 'Error: docker is not installed.' >&2
    read -p "Do you want to install it? (y/N)" choice
    case "$choice" in 
    y|Y ) install_docker;;
    * ) echo "Exiting" && exit 1;;
    esac
fi

# Checking if docker image is found
if [ -z "$(docker images | grep beamformer)" ]; then
    echo "Docker image not found."
    read -p "Do you want to build it? (y/N)" choice
    case "$choice" in 
    y|Y ) docker build -t beamformer .;;
    * ) echo "Exiting" && exit 1;;
    esac
fi


if [ -z "$(docker images | grep tftpd)" ]; then
    echo -e "$Blue[INFO]$Color_Off System does not have a TFTP-server image for the FPGA to boot from"
    echo "Please see boot/README.md"
fi

# Starting the container
docker run -it \
    $RT_ARGS \
    -v $(pwd):/usr/src/app \
    -e DISPLAY=$DISPLAY  \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --user=$(id -u $USER) \
    beamformer bash







