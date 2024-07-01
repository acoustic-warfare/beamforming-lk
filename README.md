# Beamformer 

This is an adaptation of the beamformer-pipeline for _

## Installation

Clone this project

    git clone https://github.com/acoustic-warfare/beamforming-lk.git

Go to the directory

    cd beamforming-lk

## Prerequisites

The program requires some modules to run: 

* ***Eigen3*** Linear algebra and vector ops. (https://eigen.tuxfamily.org)

* ***OpenCV2*** Camera feed and application window (https://opencv.org/)

* ***RtAudio*** Audio playback (https://www.music.mcgill.ca/~gary/rtaudio/)

You may also use `Doxygen` to compile the documentation, see Documentation.

## Prerequisites (Docker)

The project may also be compiled and executed inside a Docker environment. A working Docker installation is required and Docker without root privileges is recommended. 

    docker build -t beamformer .

## Configuration

Configurations can be made to the `config.yaml` file. All settings will be converted to config files in respective languages in `src/`. *do not edit any other config.\* file* since they will be overwritten on next build.

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

## Testing

Run the automated tests 

    make test


## Documentation 

You may see a structural documentation of the project by building the `Doxygen`
page which will generate a documentation page at `doc/html/index.html`

    make doc

## Offline usage 
You may want to test the system without an FPGA connected, you may look at the
`udp/` directory for further information. 

## Booting the FPGA
A simple TFTP boot server is given in the directory `boot/`. This must be built manually. See [`boot/README.md`](https://github.com/acoustic-warfare/beamforming-lk/tree/main/boot) for more information





