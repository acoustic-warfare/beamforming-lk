cmake_minimum_required(VERSION 3.10.1)
project(beamformer VERSION 1.0.1)


# Optimizations
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

IF (NOT PYTHON)
    find_program(PYTHON "python3")
ENDIF ()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -mavx2 -O3")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build")


# ---- Configuration ----
set(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/config.yaml)
set(CONFIG_MAKER "${CMAKE_CURRENT_SOURCE_DIR}/scripts/config_maker.py")

add_custom_command(
        DEPENDS ${CONFIG_FILE}

        # Generate Cython module/src/config.pxd
        COMMAND ${PYTHON} ${CONFIG_MAKER} ${CONFIG_FILE} ${CMAKE_CURRENT_SOURCE_DIR}/module/src --cython
        OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/module/src/config.pxd

        # Generate Python build/config.py
        COMMAND ${PYTHON} ${CONFIG_MAKER} ${CONFIG_FILE} ${CMAKE_CURRENT_BINARY_DIR} --python
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/config.py
        # cp ${CMAKE_CURRENT_BINARY_DIR}/config.py ${CMAKE_CURRENT_SOURCE_DIR}/module/config.py
        #

        OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/module/config.py
        COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_BINARY_DIR}/config.py
        ${CMAKE_CURRENT_SOURCE_DIR}/module/config.py

        #DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${MY_RESOURCE_FILE}
        # OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/module/config.py
        COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/__init__.py

        # Generate C/C++ config.h
        COMMAND ${PYTHON} ${CONFIG_MAKER} ${CONFIG_FILE} ${CMAKE_CURRENT_SOURCE_DIR}/src --c
        OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/src/config.h
)

set(CONFIGURATIONS
        ${CONFIG_FILE}
        ${CMAKE_CURRENT_BINARY_DIR}/config.py
        ${CMAKE_CURRENT_SOURCE_DIR}/src/config.h
        ${CMAKE_CURRENT_SOURCE_DIR}/module/src/config.pxd
)

add_custom_target(config DEPENDS ${CONFIGURATIONS})


# ---- Cython module ----
set(CMAKE_MODULE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/module")
set(SETUP_PY_IN "${CMAKE_MODULE_DIR}/setup.py.in")
set(SETUP_PY "${CMAKE_MODULE_DIR}/setup.py")
set(PY_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/build/pytimestamp")


configure_file(
        ${SETUP_PY_IN}
        ${SETUP_PY}
)

# Define the Python virtual environment directory
set(PYTHON_VENV_DIR "${CMAKE_MODULE_DIR}/venv")

# Define the command to create the virtual environment
set(CREATE_VENV_COMMAND "${PYTHON}" -m venv "${PYTHON_VENV_DIR}")

# Define the commands to install dependencies inside the virtual environment
set(INSTALL_DEPENDENCIES_COMMAND
        "${PYTHON_VENV_DIR}/bin/pip" install -r ${CMAKE_MODULE_DIR}/requirements.txt
        COMMENT "Done installing "
)

set(PYTHON "${PYTHON_VENV_DIR}/bin/python")

# Define a custom target to create the virtual environment and install dependencies
add_custom_command(
        OUTPUT ${PYTHON_VENV_DIR}
        COMMAND ${CREATE_VENV_COMMAND}
        COMMAND ${INSTALL_DEPENDENCIES_COMMAND}
        COMMENT "Creating Python virtual environment and installing dependencies"
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
)

set(MODULE_SOURCES
        ${CMAKE_MODULE_DIR}/src/config.pxd
        ${CMAKE_MODULE_DIR}/src/antenna.pyx
        ${CMAKE_MODULE_DIR}/src/antenna.pxd
        ${CMAKE_MODULE_DIR}/src/eigen.pxd
        ${CMAKE_MODULE_DIR}/src/pipeline.pyx
        ${CMAKE_MODULE_DIR}/src/pipeline.pxd
)


add_custom_command(DEPENDS ${PYTHON_VENV_DIR} ${MODULE_SOURCES}
        OUTPUT "${PY_OUTPUT}"

        # Build modules
        COMMENT "Starting build"
        COMMAND ${PYTHON} ${SETUP_PY} build_ext
        #COMMAND ${PYTHON_VENV_DIR}/bin/python ${SETUP_PY} build_ext 
        # --build-lib=lib --build-temp=build

        # Generate timestamp
        COMMAND ${CMAKE_COMMAND} -E touch ${PY_OUTPUT}
)


add_custom_target(module DEPENDS ${PY_OUTPUT})


# ---- Program ----
# Find packages


# Matrix operations
find_package(Eigen3 REQUIRED)

# MQTT Communication
find_package(WaraPSClient REQUIRED)


# Generating UUID:s
find_package(Boost REQUIRED)


# Find OpenCV package
find_package(OpenCV REQUIRED)

# Find GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0>=1.0 gstreamer-base-1.0>=1.0)
pkg_check_modules(DYNRT REQUIRED dynrt)


find_library(portaudio
        portaudio
        REQUIRED
)

find_library(libgps
       gps
       REQUIRED
)

# Include directories for OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})

# Include directories for GStreamer
include_directories(${GSTREAMER_INCLUDE_DIRS})

# Add the required libraries
link_directories(${GSTREAMER_LIBRARY_DIRS})

# # Configuration maker
# add_custom_target(config #ALL DEPENDS
#   
#   # Generate C/C++ config.h
#   COMMAND ${PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/config_maker.py ${CMAKE_CURRENT_SOURCE_DIR}/config.yaml ${CMAKE_CURRENT_SOURCE_DIR}/src --c
#
#   # Python 
#   COMMAND ${PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/config_maker.py ${CMAKE_CURRENT_SOURCE_DIR}/config.yaml ${CMAKE_CURRENT_BINARY_DIR} --python
#
# )
#  add_executable(audio 
# ${CMAKE_CURRENT_SOURCE_DIR}/src/audio.cpp
# )
#
#
# target_link_libraries(audio PkgConfig::RtAudio)

set(SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/receiver.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/antenna.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/pipeline.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/delay.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/options.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms/pso_seeker.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms/gradient_ascend.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms/mimo.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/awpu.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/geometry.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/audio/audio_wrapper.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/aw_control_unit.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms/triangulate.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/WaraPS/target_handler.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/rtmp.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/rtmp.hpp
)


# Main program
add_executable(beamformer
        ${CMAKE_CURRENT_SOURCE_DIR}/src/config.h
        ${SOURCES}
        ${GSTREAMER}
)

# Linking
target_link_libraries(beamformer
        Eigen3::Eigen
        ${OpenCV_LIBS}
        portaudio
        gps
        WaraPSClient::WaraPSClient
        ${GSTREAMER_LIBS}
)

target_include_directories(beamformer PUBLIC ${Boost_INCLUDE_DIRS})


add_executable(awpu
        ${CMAKE_CURRENT_SOURCE_DIR}/src/config.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/receiver.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/antenna.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/pipeline.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/delay.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/options.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms/pso_seeker.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms/gradient_ascend.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms/mimo.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/awpu.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/awpu_main.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/geometry.cpp
)

# Linking
target_link_libraries(awpu
        Eigen3::Eigen
        ${OpenCV_LIBS}
)


add_dependencies(beamformer config)

add_custom_target(test.udp
        COMMAND echo "Generating fake stream to localhost"
        COMMAND udpreplay -i lo ${CMAKE_CURRENT_SOURCE_DIR}/udp/converted/test.pcap
)

# # Testbench
# add_custom_target(test
#   COMMAND ${PYTHON} -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR} -vv #--failfast
# )

add_custom_target(test
        # COMMAND ${PYTHON} -m unittest discover -s ..
        # COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/tests/test.sh
        COMMAND ${PYTHON} -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR} -vv #--failfast

)

# Documentation
add_custom_target(doc
        COMMAND doxygen ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile
)
