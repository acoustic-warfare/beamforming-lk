# Matrix operations
find_package(Eigen3 REQUIRED)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Include directories for OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})

set(SOURCES 
    awpu.h
    awpu.cpp
)

add_library(awpu STATIC ${SOURCES})
target_include_directories(awpu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(awpu PRIVATE 
    geometry
    fpga
    dsp
    Eigen3::Eigen
    ${OpenCV_LIBS}
)

add_executable(awpu_test
    main.cpp
    ${SOURCES}
)
target_link_libraries(awpu_test PRIVATE
    geometry
    fpga
    dsp
    Eigen3::Eigen
    ${OpenCV_LIBS}
)

