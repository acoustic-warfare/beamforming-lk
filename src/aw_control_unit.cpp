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
        std::cout << "Continuing without WARA PS Connection" << std::endl;
        usingWaraPS_ = false;
    }

    AWProcessingUnit awpu = AWProcessingUnit("10.0.0.1", 21878);
    awpu.calibrate();
    awpu.start(PSO);

    cv::namedWindow(APPLICATION_NAME, cv::WINDOW_NORMAL);
    cv::resizeWindow(APPLICATION_NAME, APPLICATION_WIDTH, APPLICATION_HEIGHT);

    cv::Mat frame(Y_RES, X_RES, CV_8UC1);
    cv::Mat colorFrame(Y_RES, X_RES, CV_8UC1);

    // Used to time sensor publishing
    auto timer = std::chrono::system_clock::now();

    while ((usingWaraPS_ && client_.running()) || !usingWaraPS_) {
        awpu.draw_heatmap(&frame);

        cv::GaussianBlur(frame, colorFrame,
                         cv::Size(BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE), 0);
        cv::applyColorMap(colorFrame, colorFrame, cv::COLORMAP_JET);
        cv::imshow(APPLICATION_NAME, colorFrame);

        std::unique_lock lock(pauseMutex_);
        pausedCV_.wait(lock, [&] { return !paused_; });

        if(usingWaraPS_)
            publishData(timer);

        if(cv::waitKey(1) == 'q' || (usingWaraPS_ && !client_.running())) {
            break;
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

// Publishes data every second
void AWControlUnit::publishData(std::chrono::time_point<std::chrono::system_clock> &timer) {
    if(std::chrono::system_clock::now() - timer < std::chrono::seconds(1))
        return;

    timer = std::chrono::system_clock::now();

    if (usingGps_ && gps_waiting(&gpsData_, 1000)) {
        if (gps_read(&gpsData_, nullptr, 0) == -1) {
            std::cerr << "GPS Read error" << std::endl;
        } else {
            nlohmann::json gpsJson = {
                    {"longitude", std::to_string(gpsData_.fix.longitude)},
                    {"latitude",  std::to_string(gpsData_.fix.latitude)},
                    {"altitude",  std::to_string(gpsData_.fix.altitude)},
                    {"type",      "GeoPoint"}
            };
            client_.PublishMessage("sensor/position", gpsJson.dump(4));
        }
    }

    client_.PublishMessage("sensor/heading", std::to_string(90.0)); // Currently not a real value
    client_.PublishMessage("sensor/course", std::to_string(0));
    client_.PublishMessage("sensor/speed", std::to_string(0));
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