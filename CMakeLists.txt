cmake_minimum_required(VERSION 3.10.1)
project(beamforming-lk VERSION 1.0.1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -mavx2 -Ofast -save-temps -fverbose-asm -ffloat-store -ffast-math -fno-rounding-math")

# Documentation
add_custom_target(doc
        COMMAND doxygen ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile
)

add_subdirectory(src/geometry)
add_subdirectory(src/fpga)
add_subdirectory(src/dsp)
add_subdirectory(src/audio)
add_subdirectory(src/aw_processing_unit)
add_subdirectory(src/target_handler)
add_subdirectory(src/aw_control_unit)

set(SOURCES src/main.cpp)
add_executable(beamformer ${SOURCES})

target_link_libraries(beamformer 
        awcu
        awpu
        targetHandler
        gps
)