//
// Created by janne on 2024-07-18.
//

#ifndef TARGETHANDLER_H
#define TARGETHANDLER_H
#include <gps.h>
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
    WaraPSClient targetClient_ = WaraPSClient("lk_target", WARAPS_ADDRESS, std::getenv("MQTT_USERNAME"),
                                              std::getenv("MQTT_PASSWORD"));
    std::chrono::duration<double> targetUpdateInterval_ = std::chrono::milliseconds(200);

    gps_data_t *gpsData_;

    std::vector<AWProcessingUnit *> awpus_;
    double minProbability_ = 0.5;

    std::thread workerThread_;
    std::mutex mutex_;
    std::shared_ptr<bool> running = std::make_shared<bool>(false);

public:
    explicit TargetHandler(gps_data_t *gpsData) : gpsData_(gpsData) {}

    ~TargetHandler();

    void Start();

    void Stop();

    void SetSensitivity(double sensitivity);

    std::vector<Eigen::Vector3d> getTargets();

    TargetHandler &operator<<(AWProcessingUnit *awpu);

    void FindTargets(const std::vector<Target> &targets);

    void DisplayTarget(bool toggle);

    Eigen::Vector3d getLoudestTarget();
};


#endif //TARGETHANDLER_H
