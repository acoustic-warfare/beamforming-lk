/** @file target_handler.cpp
 * @author Janne
 * @date 2024-07-18
*/

#include "target_handler.h"

#include <ranges>

#include "../algorithms/triangulate.h"

void TargetHandler::Stop() {
    *running = false;
    workerThread_.join();
    if(debugLogging_)
        logFile_.close();

    DisplayTarget(false);
}

TargetHandler::~TargetHandler() {
    Stop();
}

void TargetHandler::Start() {
    if(debugLogging_)
        logFile_.open("Targets.txt");
    *running = true;
    workerThread_ = std::thread([&] {
        while (*running) {
            std::vector<std::vector<Target> > targets;
            for (const auto awpu: awpus_) {
                targets.push_back(awpu->targets());
            }
            FindIntersects(targets);
            UpdateTracks();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
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

void TargetHandler::FindIntersects(std::vector<std::vector<Target> > &targets) {
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

    FindIntersectsRecursively(translatedTargets[0], translatedTargets.begin() + 1, translatedTargets.end(), foundTargets);
}

void TargetHandler::UpdateTracks() {
    int bestTrackHits = -1;
    for (auto &track: tracks_) {
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - track.timeLastHit).
            count() > 1) {
            track.valid = false;
            continue;
        }

        if (track.hits > bestTrackHits) {
            bestTrack_ = track;
            bestTrackHits = track.hits;
        }
    }
}

void TargetHandler::CheckTracksForTarget(TriangulatedTarget &target) {
    int invalidTrackIndex = -1;
    for (int i = 0; i < tracks_.size(); ++i) {
        if (!tracks_[i].valid) {
            invalidTrackIndex = i;
            continue;
        }

        // If two track hits are too close to each other, it's usually not a sound target (pun not intended)

        if (constexpr double targetMinimumDistance = 1e-15;
            abs(target.position.x() - tracks_[i].position.x()) < targetMinimumDistance &&
            abs(target.position.y() - tracks_[i].position.y()) < targetMinimumDistance &&
            abs(target.position.z() - tracks_[i].position.z()) < targetMinimumDistance) {
            return;
        }

        if (constexpr double targetMaximumDistance = 1;
            abs(target.position.x() - tracks_[i].position.x()) < targetMaximumDistance &&
            abs(target.position.y() - tracks_[i].position.y()) < targetMaximumDistance &&
            abs(target.position.z() - tracks_[i].position.z()) < targetMaximumDistance) {
            tracks_[i].position = target.position;
            tracks_[i].hits++;
            return;
        }
    }

    if (invalidTrackIndex != -1) {
        tracks_[invalidTrackIndex] = {target.position, std::chrono::steady_clock::now(), true, 1};
        return;
    }

    tracks_.emplace_back(target.position, std::chrono::steady_clock::now(), true, 1);
}

void TargetHandler::FindIntersectsRecursively(std::vector<CartesianTarget> &toCompare, // NOLINT(*-no-recursion)
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


                logFile_ << target.directionLine.origin().transpose() << "," << target.directionLine.direction().
                        transpose() << ";"
                        << otherTarget.directionLine.origin().transpose() << "," << otherTarget.directionLine.
                        direction().transpose() << std::endl;

                Eigen::Vector3d intersection = triangulatePoint(target.directionLine, otherTarget.directionLine);

                if (debugLogging_) {
                    logFile_ << target.directionLine.origin().transpose() << "," << target.directionLine.direction().
                            transpose() << ";"
                            << otherTarget.directionLine.origin().transpose() << "," << otherTarget.directionLine.
                            direction().transpose() << std::endl;
                }

                if (intersection.norm() == 0 || intersection.norm() > 50) {
                    continue;
                }


                TriangulatedTarget newTarget{
                    intersection, (otherTarget.power + target.power) / 2,
                    otherTarget.startTime.time_since_epoch().count() + target.startTime.time_since_epoch().count()
                };

                CheckTracksForTarget(newTarget);
            }
            std::advance(it, 1);
        }
    }
    FindIntersectsRecursively(*begin, begin + 1, end, out);
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
            std::this_thread::sleep_for(waraPSUpdateInterval_);

            if (!isfinite(gpsData_->fix.latitude) ||
                !isfinite(gpsData_->fix.longitude) ||
                !isfinite(gpsData_->fix.altitude)) {
                continue;
            }


            Eigen::Vector3d outPosition{
                bestTrack_.position.x(), bestTrack_.position.z(), bestTrack_.position.y()
            };

            std::cout << "Best target: " << outPosition.transpose() << " Hits: " << bestTrack_.hits << std::endl;

            const nlohmann::json targetJson = PositionToGPS(outPosition, *gpsData_);

            targetClient_.PublishMessage("sensor/position", targetJson.dump());
        }
    });
}

Eigen::Vector3d TargetHandler::getBestTarget() const {
    return bestTrack_.position;
}
