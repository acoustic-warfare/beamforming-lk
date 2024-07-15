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

    while ((usingWaraPS_ && client_.running()) || !usingWaraPS_) {
        std::unique_lock lock(pauseMutex_);
        pausedCV_.wait(lock, [&] { return !paused_; });

        if (usingGps_ && gps_waiting(&gpsData_, 1000) && usingWaraPS_) {
            if (gps_read(&gpsData_, nullptr, 0) == -1) {
                std::cerr << "GPS Read error" << std::endl;
            } else {
                sendGpsData(gpsData_);
            }
        }
    }

    if (usingGps_) {
        gps_stream(&gpsData_, WATCH_DISABLE, nullptr);
        gps_close(&gpsData_);
    }
    if (usingWaraPS_) {
        client_.Stop();
    }
}

AWControlUnit::AWControlUnit() : client_(WARAPS_NAME, WARAPS_ADDRESS, std::getenv("MQTT_USERNAME"), std::getenv("MQTT_PASSWORD")) {
    int gpsError = gps_open(GPS_ADDRESS, std::to_string(GPS_PORT).c_str(), &gpsData_);
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

    client_.SetCommandCallback("unpause", [&](const nlohmann::json &payload) {
        std::unique_lock lock(pauseMutex_);
        paused_ = true;
        lock.unlock();
        pausedCV_.notify_all();
    });
}

void AWControlUnit::sendGpsData(gps_data_t data) {
    nlohmann::json gpsJson = {
            {"longitude", std::to_string(data.fix.longitude)},
            {"latitude",  std::to_string(data.fix.latitude)},
            {"altitude",  std::to_string(data.fix.altitude)},
            {"type",      "GeoPoint"}
    };
    client_.PublishMessage("sensor/position", gpsJson.dump(4));
}
