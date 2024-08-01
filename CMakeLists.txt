cmake_minimum_required(VERSION 3.10.1)
project(beamforming-lk VERSION 1.0.1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -mavx2 -Ofast -save-temps -fverbose-asm -ffloat-store -ffast-math -fno-rounding-math")

set(SOURCES)

add_subdirectory(src)

# Documentation
add_custom_target(doc
        COMMAND doxygen ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile
)

add_subdirectory(src/geometry)
add_subdirectory(src/fpga)
add_subdirectory(src/dsp)
add_subdirectory(src/audio)
add_subdirectory(src/awpu)
add_subdirectory(src/algorithms)
add_subdirectory(src/TargetHandler)
add_subdirectory(src/awcu)