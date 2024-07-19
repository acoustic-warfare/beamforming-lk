//
// Created by janne on 2024-07-18.
//

#include "TargetHandler.h"

#include "../algorithms/triangulate.h"

#define LK_DISTANCE 6 // TODO: Fixa mer konkret värde?

void TargetHandler::Stop() {
    *running = false;
    workerThread_.join();
}

void TargetHandler::Start() {
    *running = true;

    workerThread_ = std::thread([&] {
        while (*running) {
            std::vector<Target> targets;
            for (const auto awpu: awpus_) {
                for (auto [direction, power, probability]: awpu->targets()) {
                    targets.emplace_back(direction, power, probability);
                }
            }
            findTargets(targets);
        }
    });
}


void TargetHandler::SetSensitivity(const double sensitivity) {
    min_probability_ = sensitivity;
}

TargetHandler &TargetHandler::operator<<(AWProcessingUnit *awpu) {
    awpus_.push_back(awpu);
    return *this;
}

void TargetHandler::findTargets(const std::vector<Target> &targets) {
    std::vector<Eigen::Vector3d> foundTargets;

    for (const auto target: targets) {
        if (target.probability < min_probability_)
            continue;
        for (const auto target2: targets) {
            if (target == target2 || target2.probability < min_probability_) {
                continue;
            }
            Eigen::Vector3d foundPoint =
                    calculateRelativePoint(target.direction.toCartesian(), target2.direction.toCartesian(),
                                           LK_DISTANCE);

            if (foundPoint.norm() > 50) {
                continue;
            }
            foundTargets.push_back(foundPoint);
        }
    }
    mutex_.lock();
    targets_ = std::move(foundTargets);
    mutex_.unlock();
}
