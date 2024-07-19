//
// Created by janne on 2024-07-04.
//

#include "aw_control_unit.h"

#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <stdexcept>

//#include "audio/audio_wrapper.h"

void AWControlUnit::Start() {
    try {
        client_.Start();
        usingWaraPS_ = true;
    } catch (std::runtime_error &e) {
        std::cerr << "WARA PS Connection error: " << e.what() << std::endl;
        std::cout << "Continuing without WARA PS Connection" << std::endl;
        usingWaraPS_ = false;
    }

    AWProcessingUnit awpu = AWProcessingUnit("10.0.0.1", 21875);
    awpu.calibrate();
    awpu.start(PSO);

    if (USE_AUDIO) {
        awpu.play_audio();
    }

    cv::namedWindow(APPLICATION_NAME, cv::WINDOW_NORMAL);
    cv::resizeWindow(APPLICATION_NAME, APPLICATION_WIDTH, APPLICATION_HEIGHT);

    cv::Mat frame(Y_RES, X_RES, CV_8UC1);
    cv::Mat colorFrame(Y_RES, X_RES, CV_8UC1);

    if (usingWaraPS_)
        data_thread_ = std::thread([this] {
            while (client_.running()) {
                publishData();
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        });

    while ((usingWaraPS_ && client_.running()) || !usingWaraPS_) {
        awpu.draw_heatmap(&frame);

        cv::GaussianBlur(frame, colorFrame,
                         cv::Size(BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE), 0);
        cv::applyColorMap(colorFrame, colorFrame, cv::COLORMAP_JET);
        cv::imshow(APPLICATION_NAME, colorFrame);

        std::unique_lock lock(pauseMutex_);
        pausedCV_.wait(lock, [&] { return !paused_; });

        if (cv::waitKey(1) == 'q' || (usingWaraPS_ && !client_.running())) {
            break;
        }
    }

    if (usingGps_) {
        gps_stream(&gpsData_, WATCH_DISABLE, nullptr);
        gps_close(&gpsData_);
    }
    if (usingWaraPS_) {
        client_.Stop();
        data_thread_.join();
    }
    if (USE_AUDIO) {
        awpu.stop_audio();
    }
}

// Publishes data every second
void AWControlUnit::publishData() {
    if (usingGps_ && gps_waiting(&gpsData_, 1000)) {
        if (gps_read(&gpsData_, nullptr, 0) == -1) {
            std::cerr << "GPS Read error" << std::endl;
        } else if ((isfinite(gpsData_.fix.altitude) &&
                    isfinite(gpsData_.fix.latitude) &&
                    isfinite(gpsData_.fix.longitude))) {// Sending NaN breaks WARA PS Arena

            nlohmann::json gpsJson = {
                    {"longitude", std::to_string(gpsData_.fix.longitude)},
                    {"latitude", std::to_string(gpsData_.fix.latitude)},
                    {"altitude", std::to_string(gpsData_.fix.altitude)},
                    {"type", "GeoPoint"}};
            client_.PublishMessage("sensor/position", gpsJson.dump(4));
        }
    }

    client_.PublishMessage("sensor/heading", std::to_string(90.0));// Currently not a real value
    client_.PublishMessage("sensor/course", std::to_string(0));
    client_.PublishMessage("sensor/speed", std::to_string(0));
    client_.PublishMessage("sensor/camera_tags", "[ \"LJUDKRIGET\" ]");
}

AWControlUnit::AWControlUnit() : client_(WARAPS_NAME, WARAPS_ADDRESS,
                                         std::getenv("MQTT_USERNAME") == nullptr ? "" : std::getenv("MQTT_USERNAME"),
                                         std::getenv("MQTT_PASSWORD") == nullptr ? "" : std::getenv("MQTT_PASSWORD")) {
    int gpsError = gps_open(GPS_ADDRESS, std::to_string(GPS_PORT).c_str(), &gpsData_);
    gpsError |= gps_stream(&gpsData_, WATCH_ENABLE | WATCH_JSON, nullptr);

    if (gpsError != 0) {
        std::cerr << "GPS Error: " << gps_errstr(gpsError) << std::endl;
        std::cerr << "Continuing without gps" << std::endl;
        usingGps_ = false;
    } else {
        usingGps_ = true;
    }
}