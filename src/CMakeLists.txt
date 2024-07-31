cmake_minimum_required(VERSION 3.10.1)
project(beamforming-lk VERSION 1.0.1)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -mavx2 -Ofast -save-temps -fverbose-asm -ffloat-store -ffast-math -fno-rounding-math")


add_subdirectory(geometry)
add_subdirectory(fpga)
add_subdirectory(dsp)
add_subdirectory(awpu)