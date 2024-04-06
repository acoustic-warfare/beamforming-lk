cmake_minimum_required(VERSION 3.10.1)
project(beamformer)


# Optimizations
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

IF(NOT PYTHON)
    find_program(PYTHON "python")
ENDIF()


set(CMAKE_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build")


# Find packages


# Matrix operations
find_package(Eigen3 REQUIRED)


# Find OpenCV package
find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(RtAudio REQUIRED IMPORTED_TARGET rtaudio)

# Include directories for OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})

# Configuration maker
add_custom_target(config #ALL DEPENDS
  
  # Generate C/C++ config.h
  COMMAND ${PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/config_maker.py ${CMAKE_CURRENT_SOURCE_DIR}/config.yaml ${CMAKE_CURRENT_SOURCE_DIR}/src --c

  # Python 
  COMMAND ${PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/config_maker.py ${CMAKE_CURRENT_SOURCE_DIR}/config.yaml ${CMAKE_CURRENT_BINARY_DIR} --python

)
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
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ring_buffer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/beamformer.cpp
  )

# Main program
add_executable(beamformer
  ${SOURCES}
)




# Linking
target_link_libraries(beamformer
    Eigen3::Eigen
    PkgConfig::RtAudio
    ${OpenCV_LIBS}
)

add_custom_target(test.udp
  COMMAND echo "Generating fake stream to localhost"
  COMMAND udpreplay -i lo ${CMAKE_CURRENT_SOURCE_DIR}/udp/converted/test.pcap
)

# Testbench
add_custom_target(test
  COMMAND ${PYTHON} -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR} -vv #--failfast
)

# Documentation
add_custom_target(doc
  COMMAND doxygen ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile
)
