/** @file target_handler.cpp
 * @author Janne
 * @date 2024-07-18
*/

#include "target_handler.h"
#include "triangulate.h"

void TargetHandler::Stop() {
    *running = false;
    workerThread_.join();
    if constexpr (debugLogging_)
        logFile_.close();

    DisplayToWaraPS(false);
}

TargetHandler::~TargetHandler() {
    if (*running)
        Stop();
}

void TargetHandler::Start() {
    if constexpr (debugLogging_)
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

    FindIntersectsRecursively(translatedTargets[0], translatedTargets.begin() + 1, translatedTargets.end(),
                              foundTargets);
}

void TargetHandler::UpdateTracks() {
    int bestTrackHits = -1;
    for (auto &track: tracks_) {
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - track.timeLastHit).
            count() > 0.5l) {
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

inline double TargetHandler::CalculateDistanceThreshold(const Track &track) {
    double value = kMinTrackHitDistance + 0.325 * log(track.hits);
    if (value > kMaxTrackHitDistance)
        value = kMaxTrackHitDistance;
    return value;
}

inline long TargetHandler::CalculateDurationThreshold(const Track &track) {
    long value = static_cast<long>(kMinTrackTimeoutTime + 0.325 * log(track.hits));
    if (value > kMaxTrackTimeoutTime)
        value = kMaxTrackTimeoutTime;
    return value;
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
                Eigen::Vector3d intersection = triangulatePoint(target.directionLine, otherTarget.directionLine, 1);

                if constexpr (debugLogging_) {
                    logFile_ << target.directionLine.origin().transpose() << "," << target.directionLine.direction().
                            transpose() << ";"
                            << otherTarget.directionLine.origin().transpose() << "," << otherTarget.directionLine.
                            direction().transpose() << ";" << std::chrono::system_clock::now().time_since_epoch().
                            count() <<
                            std::endl;
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


void TargetHandler::DisplayToWaraPS(const bool toggle) {
    if (gpsData_ == nullptr)
        return;

    if (!toggle && targetThread_.joinable()) {
        targetClient_.Stop();
        targetThread_.join();
        return;
    }

    if (!toggle) {
        return;
    }

    targetClient_.Start();


    targetThread_ = std::thread([this] {
        Eigen::Matrix3d rotationMatrix;
        // Swap z and y, rotate to lk_heading
        rotationMatrix << cos(lk_heading_), 0, sin(lk_heading_),
                -sin(lk_heading_), 0, cos(lk_heading_),
                0, 1, 0;
        while (targetClient_.running()) {
            Eigen::Vector3d outPosition = rotationMatrix * bestTrack_.position;

            int validTracks = 0;

            for (const auto &track: tracks_) {
                if (!track.valid)
                    continue;
                validTracks++;
            }


            if (!isfinite(gpsData_->fix.latitude) ||
                !isfinite(gpsData_->fix.longitude) ||
                !isfinite(gpsData_->fix.altitude)) {
                continue;
            }

            std::cout << "Best position at:" << bestTrack_.position.transpose() << " With " << bestTrack_.hits <<
                    " hits" << std::endl;

            const nlohmann::json targetJson = PositionToGPS(outPosition, *gpsData_);

            targetClient_.PublishMessage("sensor/position", targetJson.dump());
            std::this_thread::sleep_for(waraPSUpdateInterval_);
        }
    });
}

Eigen::Vector3d TargetHandler::getBestTarget() const {
    return bestTrack_.position;
}
