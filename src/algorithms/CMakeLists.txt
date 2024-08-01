# Matrix operations
find_package(Eigen3 REQUIRED)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Include directories for OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})


set(ALGORITHMS_SOURCES 
    triangulate.h
    triangulate.cpp
)

set(SOURCES ${SOURCES} ${ALGORITHMS_SOURCES})

add_library(triangulate STATIC ${ALGORITHMS_SOURCES})
target_include_directories(triangulate PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(triangulate PUBLIC
    geometry
    Eigen3::Eigen
    ${OpenCV_LIBS}
)