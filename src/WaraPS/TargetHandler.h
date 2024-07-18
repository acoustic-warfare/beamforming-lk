//
// Created by janne on 2024-07-18.
//

#ifndef TARGETHANDLER_H
#define TARGETHANDLER_H
#include "../awpu.h"

/**
 * Class that classifies and draws targets to the WARA PS display.
 * Takes two or more active AWPUS as inputs and classifies the targets based on the target function
 */
class TargetHandler {
private:
    std::vector<Eigen::Vector2d> targets_;
    std::vector<AWProcessingUnit *> awpus_;
    double sensitivity_ = INFINITY;
    static constexpr double THRESHOLD = 0.5;
    std::thread workerThread_;

    std::shared_ptr<bool> running = std::make_shared<bool>(false);

public:

    void Start();

    void Stop();

    void SetSensitivity(double sensitivity);

    TargetHandler& operator<<(AWProcessingUnit *awpu);

    void findTargets(const std::vector<std::tuple<Target, int>> &targets);
};


#endif //TARGETHANDLER_H
