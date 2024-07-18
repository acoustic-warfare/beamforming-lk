//
// Created by janne on 2024-07-17.
//

#include <gps.h>

#include <utility>
#include "target.h"
#include "../config.h"


WaraPS::Target::Target(Eigen::Vector2d position, const gps_data_t &lk_position, const int index) : client(
        "lk_target_" + std::to_string(index),
        WARAPS_ADDRESS,
        std::getenv(
            "MQTT_USERNAME") ==
        nullptr
            ? ""
            : std::getenv(
                "MQTT_USERNAME"),
        std::getenv(
            "MQTT_PASSWORD") ==
        nullptr
            ? ""
            : std::getenv(
                "MQTT_PASSWORD")),
    position_(std::move(position)), lk_position_(lk_position) {
}

void WaraPS::Target::Start() {
    std::cout << "Starting data thread" << std::endl;
    client.Start();
    *running = true;
    this->data_thread_ = std::thread([&] {
        while (*running) {
            rw_mutex.lock();

            if (std::isfinite(lk_position_.fix.latitude) &&
                std::isfinite(lk_position_.fix.longitude))
                client.PublishMessage("sensor/position", PositionToGPS().dump(4));

            rw_mutex.unlock();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });
}

void WaraPS::Target::Destroy() {
    *running = false;
    data_thread_.join();
    client.Stop();
}

inline nlohmann::json WaraPS::Target::PositionToGPS() const {
    const double long_coeff = position_[0] / (111032.0 * cos(lk_position_.fix.latitude)); // 111 km per lat degree (ish)
    const double lat_coeff = position_[1] / 111032.0; // 111 km per lat degree (ish)
    const double t_lat = lk_position_.fix.latitude + lat_coeff;
    const double t_long = lk_position_.fix.longitude + long_coeff;

    return {
        {"longitude", t_long},
        {"latitude", t_lat},
        {"altitude", 0},
        {"type", "GeoPoint"}
    };
}
