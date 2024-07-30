# Beamformer 

This is an adaptation of the beamformer-pipeline for _

## Installation

Clone this project

    git clone https://github.com/acoustic-warfare/beamforming-lk.git

Go to the directory

    cd beamforming-lk

## Realtime Kernel

It is also good to have a realtime kernel. see https://ubuntu.com/pro.

When you have a real time kernel:
```bash
sudo addgroup realtime
sudo usermod -a -G realtime $(whoami)
```

Then edit the limit configuration `/etc/security/limits.conf`
```bash
# /etc/security/limits.conf
@realtime     soft    rtprio          99
@realtime     soft    priority        99
@realtime     soft    memlock     102400
@realtime     hard    rtprio          99
@realtime     hard    priority        99
@realtime     hard    memlock     102400
```
## Prerequisites (automatic)
By running the script `start.h` the prerequistes are set up, build and run automatically.

    ./start.sh

## Prerequisites (manual)

The project may also be compiled and executed inside a Docker environment manually. A working Docker installation is required and Docker without root privileges is recommended. 

#### To build the container run the following command:

    docker build -t beamformer .

#### To run the container run the following command:

    docker run -v $(pwd):/usr/src/app -e DISPLAY=$DISPLAY -it --network=host -v /tmp/.X11-unix:/tmp/.X11-unix -e PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native -v ${XDG_RUNTIME_DIR}/pulse/native:${XDG_RUNTIME_DIR}/pulse/native -v /run/dbus:/run/dbus --device /dev/snd beamformer bash

## Configuration

Configurations can be made to the `config.yaml` file. All settings will be converted to config files in respective languages in `src/`. ***Do not edit any other config.\* file*** since they will be overwritten on next build.

Configurations may need to be done for realtime sound recording in the `src/audio/daemon.conf` file. The current standards should work for most machines. 
If you are not running the project in the docker container, replace the current `/etc/pulse/daemon.conf` with the above file.

## Building

Building the project is the same for within the Docker container and normal installation. 


Create the required build directory

    mkdir build

Change directory to `build/`

    cd build

Generate the Makefile

    cmake ..

Build the program

    make

## Usage

Run the program

    ./beamformer

It will wait until an incoming stream of data arrives. If connected to a booted FPGA and arrays the system will run on this data. 

## Offline usage 
You may want to test the system without an FPGA connected, you may look at the
`udp/` directory for further information. 

## Booting the FPGA
A simple TFTP boot server is given in the directory `boot/`. This must be built manually. See [`boot/README.md`](https://github.com/acoustic-warfare/beamforming-lk/tree/main/boot) for more information

## Documentation
You may see a structural documentation of the project by building the `Doxygen`
page which will generate a documentation page at `doc/html/index.html`

    make doc




