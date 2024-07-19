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
    std::vector<Eigen::Vector3d> targets_;
    std::vector<AWProcessingUnit *> awpus_;
    double min_probability_ = 10;
    static constexpr double THRESHOLD = 10;
    std::thread workerThread_;

    std::mutex mutex_;

    std::shared_ptr<bool> running = std::make_shared<bool>(false);

public:

    void Start();

    void Stop();

    void SetSensitivity(double sensitivity);

    std::vector<Eigen::Vector3d> getTargets();

    TargetHandler& operator<<(AWProcessingUnit *awpu);

    void findTargets(const std::vector<Target> &targets);
};


#endif //TARGETHANDLER_H
