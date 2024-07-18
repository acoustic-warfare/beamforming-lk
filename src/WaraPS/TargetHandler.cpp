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
        // Lite scuffed lösning för att den inte ska leta targets på samma FPGA
        // TODO: Smartare och snyggare lösning, typ dubbelvector?
        while (*running) {
            std::vector<std::tuple<Target, int> > targets;
            int index = 0;
            for (const auto awpu: awpus_) {
                for (const auto target: awpu->targets()) {
                    targets.emplace_back(target, index);
                }
                index++;
            }
            findTargets(targets);
        }
    });
}


void TargetHandler::SetSensitivity(const double sensitivity) {
    sensitivity_ = sensitivity;
}

TargetHandler &TargetHandler::operator<<(AWProcessingUnit *awpu) {
    awpus_.push_back(awpu);
    return *this;
}

void TargetHandler::findTargets(const std::vector<std::tuple<Target, int> > &targets) {
    std::vector<Eigen::Vector2d> foundTargets;
    for (const auto [target, index1]: targets) {
        if (target.probability > sensitivity_)
            continue;
        for (const auto [target2, index2]: targets) {
            if (target == target2 || target2.probability < sensitivity_ || index1 == index2) {
                continue;
            }
            Eigen::Vector2d foundPoint =
                    calculateRelativePoint(target.direction.toCartesian(), target2.direction.toCartesian(),
                                           LK_DISTANCE);

            if (foundPoint.norm() < 0 || foundPoint.norm() > 50) {
                continue;
            }

            std::cout << "Found target at coordinates: " << foundPoint << std::endl;
            std::cout << "At distance:" << std::to_string(foundPoint.norm()) << std::endl;

            foundTargets.push_back(foundPoint);
        }
    }

    targets_ = foundTargets;
}
