message(STATUS "CMake: Geometry")

# Matrix operations
find_package(Eigen3 REQUIRED)

set(SOURCES
    pipeline.h
    pipeline.cpp
    receiver.h
    receiver.cpp
    streams.hpp
)

add_library(fpga STATIC ${SOURCES})

target_include_directories(fpga PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(fpga PRIVATE
    Eigen3::Eigen
    geometry
)

#add_executable(fpga_test
#    main.cpp
#    ${SOURCES}
#)
#target_link_libraries(fpga_test
#    Eigen3::Eigen
#)