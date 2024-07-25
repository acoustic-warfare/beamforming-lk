//
// Created by janne on 2024-07-18.
//

#include "target_handler.h"

#include <ranges>

#include "../algorithms/triangulate.h"

#define LK_DISTANCE 6 // TODO: Fixa mer konkret värde?

void TargetHandler::Stop() {
    *running = false;
    workerThread_.join();

    DisplayTarget(false);
}

TargetHandler::~TargetHandler() {
    Stop();
}

void TargetHandler::Start() {
    *running = true;
    workerThread_ = std::thread([&] {
        while (*running) {
            std::vector<std::vector<Target> > targets;
            for (AWProcessingUnit awpu: awpus_) {
                targets.push_back(awpu.targets());
            }
            FindTargets(targets);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
}


void TargetHandler::SetSensitivity(const double sensitivity) {
    minProbability_ = sensitivity;
}

std::vector<Eigen::Vector3d> TargetHandler::getTargets() {
    mutex_.lock();
    std::vector targets(targets_);
    mutex_.unlock();

    return targets;
}

TargetHandler &TargetHandler::operator<<(AWProcessingUnit &awpu) {
    awpus_.emplace_back(awpu);
    return *this;
}

void TargetHandler::FindTargets(std::vector<std::vector<Target> > &targets) {
    std::vector<Eigen::Vector3d> foundTargets;

    std::vector<std::vector<CartesianTarget> > translatedTargets;
    for (int i = 0; i < targets.size(); ++i) {
        for (auto [direction, power, probability]: targets[i]) {
            translatedTargets[i].emplace_back(direction.toCartesian() + awpu_positions_[i], power,
                                              probability);
        }
    }

    FindTargetsRecursively(translatedTargets[0], translatedTargets.begin()+1, translatedTargets.end(), foundTargets);

    mutex_.lock();
    targets_ = std::move(foundTargets);
    mutex_.unlock();
}

void TargetHandler::DisplayTarget(const bool toggle) {
    if (!toggle && targetThread_.joinable()) {
        targetClient_.Stop();
        targetThread_.join();
        return;
    }

    if (!toggle) {
        return;
    }

    targetClient_.Start();
    targetClient_.PublishMessage("sensor/position", nlohmann::json{
                                     {"longitude", 0},
                                     {"latitude", 0},
                                     {"altitude", 0},
                                     {"type", "GeoPoint"}
                                 }.dump());

    targetThread_ = std::thread([&] {
        while (targetClient_.running()) {
            std::this_thread::sleep_for(targetUpdateInterval_);
            if (!isfinite(gpsData_->fix.latitude) ||
                !isfinite(gpsData_->fix.longitude) ||
                !isfinite(gpsData_->fix.altitude)) {
                continue;
            }

            const nlohmann::json targetJson = PositionToGPS(loudestTarget_, *gpsData_);

            targetClient_.PublishMessage("sensor/position", targetJson.dump());
        }
    });
}

Eigen::Vector3d TargetHandler::getLoudestTarget() {
    return loudestTarget_;
}