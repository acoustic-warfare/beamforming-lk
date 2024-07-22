//
// Created by janne on 2024-07-18.
//

#ifndef TARGETHANDLER_H
#define TARGETHANDLER_H
#include <wara_ps_client.h>

#include "../awpu.h"

/**
 * Class that classifies and draws targets to the WARA PS display.
 * Takes two or more active AWPUS as inputs and classifies the targets based on the target function
 */
class TargetHandler {
    std::vector<Eigen::Vector3d> targets_;
    Eigen::Vector3d loudestTarget_;
    double loudestTargetPower_ = 0;
    const double targetDecay_ = 0.9;
    std::thread targetThread_;
    WaraPSClient targetClient_ = WaraPSClient("lk_target", WARAPS_ADDRESS, std::getenv("MQTT_PASSWORD"),
                                              std::getenv("MQTT_USERNAME"));

    std::vector<AWProcessingUnit *> awpus_;
    double minProbability_ = 10;
    static constexpr double THRESHOLD = 10;

    std::thread workerThread_;
    std::mutex mutex_;
    std::shared_ptr<bool> running = std::make_shared<bool>(false);

public:
    void Start();

    void Stop();

    void SetSensitivity(double sensitivity);

    std::vector<Eigen::Vector3d> getTargets();

    TargetHandler &operator<<(AWProcessingUnit *awpu);

    void FindTargets(const std::vector<Target> &targets);

    void SetTargetDisplay(bool toggle);

    Eigen::Vector3d getLoudestTarget();
};


#endif //TARGETHANDLER_H
