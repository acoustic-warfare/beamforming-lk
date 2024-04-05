# Beamformer 

This is an adaptation of the beamformer-pipeline for _

## Installation

Clone this project

    git clone https://github.com/Irreq/neural-beamformer.git

Go to the directory

    cd neural-beamformer

## Prerequisites

The program requires some modules to run: 

* ***Eigen3*** Linear algebra and vector ops. (https://eigen.tuxfamily.org)

* ***OpenCV2*** Camera feed and application window (https://opencv.org/)

* ***RtAudio*** Audio playback (TODO) (https://www.music.mcgill.ca/~gary/rtaudio/)

You may also use `Doxygen` to compile the documentation, see Documentation.

## Building

Configure your settings in `config.yaml` values here will be converted to auto-generated config files in `src/` *do not edit any other config.\* file* since they will be overwritten on next build.


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

## Booting 
A simple TFTP boot server is given in the directory `boot/`.





