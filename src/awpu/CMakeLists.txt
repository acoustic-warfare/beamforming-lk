# Matrix operations
find_package(Eigen3 REQUIRED)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Include directories for OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})

set(AWPU_SOURCES 
    awpu.h
    awpu.cpp
)

set(SOURCES ${SOURCES} ${AWPU_SOURCES})

add_library(awpu STATIC ${AWPU_SOURCES})
target_include_directories(awpu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(awpu PUBLIC 
    geometry
    fpga
    dsp
    audio
    Eigen3::Eigen
    ${OpenCV_LIBS}
)

add_executable(awpu_test
    main.cpp
    ${AWPU_SOURCES}
)
target_link_libraries(awpu_test PRIVATE
    geometry
    fpga
    dsp
    audio
    Eigen3::Eigen
    ${OpenCV_LIBS}
)

