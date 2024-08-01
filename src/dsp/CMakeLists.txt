# Matrix operations
find_package(Eigen3 REQUIRED)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Include directories for OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})

set(DSP_SOURCES 
    delay.h
    delay.cpp
    particle.h
    particle.cpp
    pso_seeker.h
    pso_seeker.cpp
    mimo.h
    mimo.cpp
    gradient_ascend.h
    gradient_ascend.cpp
)

set(SOURCES ${SOURCES} ${DSP_SOURCES})

add_library(dsp STATIC ${DSP_SOURCES})
target_include_directories(dsp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(dsp PUBLIC
    fpga
    Eigen3::Eigen
    ${OpenCV_LIBS}
)

