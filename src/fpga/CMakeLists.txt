message(STATUS "CMake: Geometry")

# Matrix operations
find_package(Eigen3 REQUIRED)

set(FPGA_SOURCES
    pipeline.h
    pipeline.cpp
    receiver.h
    receiver.cpp
    streams.hpp
)

set(SOURCES ${SOURCES} ${FPGA_SOURCES})

add_library(fpga STATIC ${FPGA_SOURCES})

target_include_directories(fpga PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(fpga PUBLIC
    Eigen3::Eigen
    geometry
)