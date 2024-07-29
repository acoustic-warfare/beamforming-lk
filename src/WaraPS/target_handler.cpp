/** @file target_handler.cpp
 * @author Janne
 * @date 2024-07-18
*/

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
            for (const auto awpu: awpus_) {
                targets.push_back(awpu->targets());
            }
            FindTargets(targets);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
}


void TargetHandler::SetSensitivity(const double sensitivity) {
    minGradient_ = sensitivity;
}

std::vector<Eigen::Vector3d> TargetHandler::getTargets() {
    mutex_.lock();
    std::vector<Eigen::Vector3d> targets;
    for (TriangulatedTarget &target: targets_) {
        targets.push_back(target.position);
    }
    mutex_.unlock();

    return targets;
}

TargetHandler &TargetHandler::AddAWPU(AWProcessingUnit *awpu, const Eigen::Vector3d &position) {
    awpus_.emplace_back(awpu);
    awpu_positions_.emplace_back(position);
    return *this;
}

void TargetHandler::FindTargets(std::vector<std::vector<Target> > &targets) {
    std::vector<TriangulatedTarget> foundTargets;
    std::vector<std::vector<CartesianTarget> > translatedTargets{targets.size()};

    for (int i = 0; i < targets.size(); ++i) {
        translatedTargets[i].reserve(targets[i].size());
        for (auto [direction, power, probability]: targets[i]) {
            CartesianTarget t{{awpu_positions_[i], direction.toCartesian()}, power, probability};
            translatedTargets[i].emplace_back(std::move(t));
        }
    }

    FindTargetsRecursively(translatedTargets[0], translatedTargets.begin() + 1, translatedTargets.end(), foundTargets);

    mutex_.lock();
    targets_ = std::move(foundTargets);
    mutex_.unlock();
}

void TargetHandler::FindTargetsRecursively(std::vector<CartesianTarget> &toCompare, // NOLINT(*-no-recursion)
                                           std::vector<std::vector<CartesianTarget> >::iterator begin,
                                           std::vector<std::vector<CartesianTarget> >::iterator end,
                                           std::vector<TriangulatedTarget> &out) {
    if (begin == end) {
        return;
    }


    // invariant: out will contain all viable intersections between toCompare and all vectors from begin -> it
    for (CartesianTarget &target: toCompare) {
        auto it = begin;
        while (it != end) {
            for (CartesianTarget &otherTarget: *it) {
                if (target.gradient > minGradient_ || otherTarget.gradient > minGradient_) {
                    continue;
                }

                Eigen::Vector3d intersection = triangulatePoint(target.directionLine, otherTarget.directionLine);

                if (intersection.norm() == 0 || intersection.norm() > 50) {
                    continue;
                }

                loudestTarget_.powerAverage = loudestTarget_.powerAverage * targetDecay_ + pow(
                                                  otherTarget.power + target.power, 2) * (1 - targetDecay_);

                if (pow(otherTarget.power + target.power, 5) < loudestTarget_.powerAverage) {
                    loudestTarget_ = {intersection, pow(otherTarget.power + target.power, 5), std::chrono::seconds(0)};
                    continue;
                }

                out.emplace_back(intersection, sqrt(pow(target.power, 5) + pow(otherTarget.power, 5)),
                                 std::chrono::seconds(0));
            }
            std::advance(it, 1);
        }
    }
    FindTargetsRecursively(*begin, begin + 1, end, out);
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
            std::cout << loudestTarget_.position.transpose() << std::endl;
            if (!isfinite(gpsData_->fix.latitude) ||
                !isfinite(gpsData_->fix.longitude) ||
                !isfinite(gpsData_->fix.altitude)) {
                continue;
            }

            const nlohmann::json targetJson = PositionToGPS(loudestTarget_.position, *gpsData_);

            targetClient_.PublishMessage("sensor/position", targetJson.dump());
        }
    });
}

Eigen::Vector3d TargetHandler::getLoudestTarget() const {
    return loudestTarget_.position;
}
