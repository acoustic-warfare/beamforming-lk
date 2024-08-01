# MQTT Communication
find_package(WaraPSClient REQUIRED)

# Generating UUID:s
find_package(Boost REQUIRED)

set(WARAPS_SOURCES 
    target_handler.h
    target_handler.cpp
)

set(SOURCES ${SOURCES} ${WARAPS_SOURCES})

add_library(waraps STATIC ${WARAPS_SOURCES})
target_include_directories(waraps PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(waraps PUBLIC
    awpu
    triangulate
    WaraPSClient::WaraPSClient
)

