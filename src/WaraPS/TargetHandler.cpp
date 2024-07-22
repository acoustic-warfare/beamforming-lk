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
            FindTargets(targets);
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

TargetHandler &TargetHandler::operator<<(AWProcessingUnit *awpu) {
    awpus_.push_back(awpu);
    return *this;
}

void TargetHandler::FindTargets(const std::vector<Target> &targets) {
    std::vector<Eigen::Vector3d> foundTargets;

    for (const auto target: targets) {
        if (target.probability < minProbability_)
            continue;
        for (const auto target2: targets) {
            if (target == target2 || target2.probability < minProbability_) {
                continue;
            }
            Eigen::Vector3d foundPoint =
                    calculateRelativePoint(target.direction.toCartesian(), target2.direction.toCartesian(),
                                           LK_DISTANCE);

            if (foundPoint.norm() > 50) {
                continue;
            }

            loudestTargetPower_ = loudestTargetPower_ * targetDecay_ + (target.power + target2.power) * (
                                      1 - targetDecay_);

            if ((target.power + target2.power) / 2 > loudestTargetPower_) {
                loudestTarget_ = foundPoint;
                loudestTargetPower_ = target.power + target2.power;
            }

            foundTargets.push_back(foundPoint);
        }
    }
    mutex_.lock();
    targets_ = std::move(foundTargets);
    mutex_.unlock();
}

void TargetHandler::SetTargetDisplay(const bool toggle) {
    if(!toggle && targetThread_.joinable()) {
        targetClient_.Stop();
        targetThread_.join();
        return;
    }
    if(!toggle) {
        return;
    }

    targetClient_.Start();

    targetThread_ = std::thread([&] {
        nlohmann::json targetJson = PositionToGPS(loudestTarget_);
    });
}

Eigen::Vector3d TargetHandler::getLoudestTarget() {
    return loudestTarget_;
}
