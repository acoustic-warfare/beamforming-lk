find_library(libgps
        gps
        REQUIRED
)

set(AWCU_SOURCES
        aw_control_unit.h
        aw_control_unit.cpp
)

set(SOURCES ${SOURCES} ${AWCU_SOURCES})

add_library(awcu STATIC ${AWCU_SOURCES})
target_include_directories(awcu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(awcu PUBLIC
        awpu
        targetHandler
        gps
)
