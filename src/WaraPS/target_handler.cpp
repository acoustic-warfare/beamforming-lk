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
            for (const auto awpu: awpus_) {
                targets.push_back(awpu->targets());
            }
            FindTargets(targets);
            UpdateTracks();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });
}


void TargetHandler::SetSensitivity(const double sensitivity) {
    minGradient_ = sensitivity;
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
        for (auto target: targets[i]) {
            CartesianTarget t{
                {awpu_positions_[i], target.direction.toCartesian() / target.direction.toCartesian().norm()},
                target.power, target.probability, target.start
            };
            translatedTargets[i].emplace_back(std::move(t));
        }
    }

    FindTargetsRecursively(translatedTargets[0], translatedTargets.begin() + 1, translatedTargets.end(), foundTargets);
}

double TargetHandler::CalculateTargetWeight(const TriangulatedTarget &triangulated_target) {
    return triangulated_target.powerAverage;
}

void TargetHandler::UpdateTracks() {
    int bestTrackHits = -1;
    for (auto &track: tracks_) {
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - track.timeLastHit).count() > 2) {
            track.valid = false;
            continue;
        }

        if (track.hits > bestTrackHits) {
            bestTarget_ = track.position;
            bestTrackHits = track.hits;
        }
    }
}

void TargetHandler::CheckTracksForTarget(TriangulatedTarget &target) {
    int invalidTrackIndex = -1;
    for (int i = 0; i < tracks_.size(); ++i) {
        if(!tracks_[i].valid) {
            invalidTrackIndex = i;
            continue;
        }
        if (constexpr double targetAcceptableDistance = 1;
        abs(target.position.x() - tracks_[i].position.x()) < targetAcceptableDistance &&
            abs(target.position.y() - tracks_[i].position.y()) < targetAcceptableDistance &&
            abs(target.position.z() - tracks_[i].position.z()) < targetAcceptableDistance) {
            tracks_[i].position = target.position;
            tracks_[i].hits++;
            return;
        }
    }

    if(invalidTrackIndex != -1) {
        tracks_[invalidTrackIndex] = {target.position, std::chrono::steady_clock::now(), true, 1};
        return;
    }

    tracks_.emplace_back(target.position, std::chrono::steady_clock::now(), true, 1);
}

void TargetHandler::FindTargetsRecursively(std::vector<CartesianTarget> &toCompare, // NOLINT(*-no-recursion)
                                           const std::vector<std::vector<CartesianTarget> >::iterator begin,
                                           const std::vector<std::vector<CartesianTarget> >::iterator end,
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

                std::cout << "Intersection: " << intersection.transpose() << std::endl;

                TriangulatedTarget newTarget{
                    intersection, (otherTarget.power + target.power) / 2,
                    (otherTarget.startTime.time_since_epoch().count() + target.startTime.time_since_epoch().count())
                };

                CheckTracksForTarget(newTarget);
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
            if (!isfinite(gpsData_->fix.latitude) ||
                !isfinite(gpsData_->fix.longitude) ||
                !isfinite(gpsData_->fix.altitude)) {
                continue;
            }

            Eigen::Vector3d outPosition{
                bestTarget_.x(), bestTarget_.z(), bestTarget_.y()
            };

            std::cout << "Best target: " << outPosition.transpose() << std::endl;

            const nlohmann::json targetJson = PositionToGPS(outPosition, *gpsData_);

            targetClient_.PublishMessage("sensor/position", targetJson.dump());
        }
    });
}

Eigen::Vector3d TargetHandler::getBestTarget() const {
    return bestTarget_;
}
