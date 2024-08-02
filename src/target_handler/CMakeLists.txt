# MQTT Communication
find_package(WaraPSClient REQUIRED)

# Generating UUID:s
find_package(Boost REQUIRED)

set(TARGET_HANDLER_SOURCES
        target_handler.cpp
        triangulate.cpp
)

set(SOURCES ${SOURCES} ${TARGET_HANDLER_SOURCES})

add_library(targetHandler STATIC ${TARGET_HANDLER_SOURCES})
target_include_directories(targetHandler PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(targetHandler PUBLIC
        awpu
        WaraPSClient::WaraPSClient
)

