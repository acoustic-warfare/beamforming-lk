cmake_minimum_required(VERSION 3.10.1)
project(beamformer)
project(First VERSION 0.0.1)


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# set(CMAKE_CXX_FLAGS "-Wall -Wextra")
# set(CMAKE_CXX_FLAGS_DEBUG "-g")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3")

IF(NOT PYTHON)
    find_program(PYTHON "python")
ENDIF()

set(CMAKE_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build")
set(SETUP_PY_IN "${CMAKE_CURRENT_SOURCE_DIR}/module/setup.py.in")
set(SETUP_PY    "${CMAKE_CURRENT_BINARY_DIR}/module/setup.py")

set(PY_OUTPUT      "${CMAKE_CURRENT_BINARY_DIR}/build/pytimestamp")



# Find OpenCV package
find_package(OpenCV REQUIRED)

# Include directories for OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})

# For matrix/vector
find_package(Eigen3 REQUIRED)

find_package(TBB REQUIRED)
# find_package(rtaudio REQUIRED)
find_package(RtAudio REQUIRED)


add_custom_target(config ALL DEPENDS
    # Generate C/C++ config.h
    COMMAND ${PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/config_maker.py
    ${CMAKE_CURRENT_SOURCE_DIR}/config.yaml ${CMAKE_CURRENT_SOURCE_DIR}/src --c
  
    # Generate python config.py
    COMMAND ${PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/config_maker.py
    ${CMAKE_CURRENT_SOURCE_DIR}/config.yaml ${CMAKE_CURRENT_SOURCE_DIR}/scripts --python

    COMMAND ${PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/config_maker.py
    ${CMAKE_CURRENT_SOURCE_DIR}/config.yaml ${CMAKE_CURRENT_SOURCE_DIR}/module --cython
   )


 configure_file(
    ${SETUP_PY_IN}
    ${SETUP_PY}
)

# Python bindings
add_custom_command(OUTPUT "${PY_OUTPUT}"
                   COMMAND ${PYTHON} ${SETUP_PY} build_ext --build-lib=lib --build-temp=build
                   COMMAND ${CMAKE_COMMAND} -E touch ${PY_OUTPUT}
               )

add_custom_target(cython_binds ALL DEPENDS ${PY_OUTPUT})






 add_executable(audio 
${CMAKE_CURRENT_SOURCE_DIR}/src/audio.cpp
# ${CMAKE_CURRENT_SOURCE_DIR}/lib/RtAudio/RtAudio.h
# ${CMAKE_CURRENT_SOURCE_DIR}/lib/RtAudio/RtAudio.cpp
#${CMAKE_CURRENT_SOURCE_DIR}/lib/rtaudio/RtAudio.h

#[[ ${CMAKE_CURRENT_SOURCE_DIR}/lib/rtaudio/RtAudio.cpp ]]

   )



find_package(PkgConfig REQUIRED)
pkg_check_modules(RtAudio REQUIRED IMPORTED_TARGET rtaudio)



target_link_libraries(audio PkgConfig::RtAudio)
#
# #[[ target_link_libraries(audio RtAudio) ]]
# Your-external "mylib", add GLOBAL if the imported library is located in directories above the current.
#add_library(rtaudio SHARED IMPORTED )
# You can define two import-locations: one for debug and one for release.
# set_target_properties(rtaudio PROPERTIES IMPORTED_LOCATION
#   ${CMAKE_CURRENT_SOURCE_DIR}/lib/rtaudio/build/librtaudio.so)
#
# target_link_libraries(audio rtaudio)

add_executable(beamformer
  ${CMAKE_CURRENT_SOURCE_DIR}/src/beamformer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/antenna.h 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/antenna.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/config.h
${CMAKE_CURRENT_SOURCE_DIR}/src/ring_buffer.h
${CMAKE_CURRENT_SOURCE_DIR}/src/ring_buffer.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/receiver.h
${CMAKE_CURRENT_SOURCE_DIR}/src/receiver.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/pipeline.h
${CMAKE_CURRENT_SOURCE_DIR}/src/pipeline.cpp
# ${CMAKE_CURRENT_SOURCE_DIR}/src/testmp.cpp


)
# Link the OpenCV libraries to the executable
target_link_libraries(beamformer ${OpenCV_LIBS})

target_link_libraries(beamformer
    Eigen3::Eigen
)

add_executable(mp

${CMAKE_CURRENT_SOURCE_DIR}/src/mp.cpp


  )
target_link_libraries(mp 
TBB::tbb
  )

# Testbench
add_custom_target(test
  COMMAND ${PYTHON} -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR} -vv #--failfast
)
# Documentation
add_custom_target(doc
  COMMAND doxygen ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile
)
# # Find the required Qt5 package
# # find_package(Qt5Widgets REQUIRED)
# # Find the required Qt5 package
# find_package(Qt5 REQUIRED COMPONENTS Widgets Multimedia MultimediaWidgets)
#
#
# set(CMAKE_AUTOMOC ON)
# set(CMAKE_AUTOUIC ON)
# set(CMAKE_AUTORCC ON)
#
# # Add your source files
# set(SOURCES
#   ${CMAKE_CURRENT_SOURCE_DIR}/src/demo.cpp
#   ${CMAKE_CURRENT_SOURCE_DIR}/src/camera/camera.h
#   ${CMAKE_CURRENT_SOURCE_DIR}/src/camera/camera.cpp
# ${CMAKE_CURRENT_SOURCE_DIR}/src/camera/imagesettings.h 
# ${CMAKE_CURRENT_SOURCE_DIR}/src/camera/imagesettings.cpp 
# ${CMAKE_CURRENT_SOURCE_DIR}/src/camera/videosettings.h 
# ${CMAKE_CURRENT_SOURCE_DIR}/src/camera/videosettings.cpp
#
#   )
#
# # Create an executable
# add_executable(QtMatrixDemo ${SOURCES})
#
# # Link against the Qt5 libraries
# target_link_libraries(QtMatrixDemo Qt5::Widgets Qt5::Multimedia
#   Qt5::MultimediaWidgets)
