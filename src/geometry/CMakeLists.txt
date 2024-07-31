message(STATUS "CMake: Geometry")

# Matrix operations
find_package(Eigen3 REQUIRED)

set(SOURCES
    antenna.h
    antenna.cpp
    geometry.h
    geometry.cpp
    kf.h
)

add_library(geometry STATIC ${SOURCES})
target_link_libraries(geometry
    Eigen3::Eigen
)
target_include_directories(geometry PUBLIC "${CURRENT_SOURCE_DIR}")

add_executable(geometry_test
    main.cpp
    ${SOURCES}
)
target_link_libraries(geometry_test
    Eigen3::Eigen
)