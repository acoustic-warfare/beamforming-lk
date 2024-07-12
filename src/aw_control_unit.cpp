//
// Created by janne on 2024-07-04.
//

#include <stdexcept>
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
#include "aw_control_unit.h"

void AWControlUnit::Start() {
    try {
        client_.Start();
        usingWaraPS_ = true;
    } catch (std::runtime_error &e) {
        std::cerr << "WARA PS Connection error: " << e.what() << std::endl;
        std::cerr << "Continuing without WARA PS Connection" << std::endl;
        usingWaraPS_ = false;
    }

    while(client_.running()) {
        std::unique_lock lock(pauseMutex_);
        pausedCV_.wait(lock, [&]{return !paused_;});

        std::cout << "plotting" << std::endl;
    }
}

AWControlUnit::AWControlUnit() : client_(WARAPS_NAME, WARAPS_ADDRESS) {
    int gpsError = gps_open(GPS_ADDRESS, reinterpret_cast<const char *>(GPS_PORT), &gpsData_);
    gpsError |= gps_stream(&gpsData_, WATCH_ENABLE | WATCH_JSON,
                           nullptr); // We only want the stream data, not pure buffer data

    if (gpsError != 0) {
        std::cerr << "GPS Error: " << gps_errstr(gpsError) << std::endl;
        std::cerr << "Continuing without gps" << std::endl;
        usingGps_ = false;
    } else {
        usingGps_ = true;
    }

    client_.SetCommandCallback("focus_bf", [&](const nlohmann::json &payload) {
        float theta = payload["theta"];
        float phi = payload["phi"];
        float duration =
                payload.contains("duration") ? (float) payload["duration"] : 5.0f;

        std::cout << "Theta: " << theta << "\nPhi: " << phi << std::endl;
        client_.PublishMessage("exec/response", std::string("Focusing beamformer for " +
                                                            std::to_string(duration)));
    });

    client_.SetCommandCallback("pause", [&](const nlohmann::json &payload) {
        std::unique_lock lock(pauseMutex_);
        paused_ = true;
    });

    client_.SetCommandCallback("unpause", [&](const nlohmann::json &payload){
        std::unique_lock lock(pauseMutex_);
        paused_ = true;
        lock.unlock();
        pausedCV_.notify_all();
    });
}
