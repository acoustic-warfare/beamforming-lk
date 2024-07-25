//
// Created by janne on 2024-07-18.
//

#ifndef TARGETHANDLER_H
#define TARGETHANDLER_H
#include <gps.h>
#include <ranges>
#include <wara_ps_client.h>

#include "../awpu.h"
#include "../kf.h"

/**
 * Class that classifies and draws targets to the WARA PS display.
 * Takes two or more active AWPUS as inputs and classifies the targets based on the target function
 */
class TargetHandler {
    struct CartesianTarget {
        Eigen::Vector3d position;
        double power;
        double probability;
    };

    std::vector<Eigen::Vector3d> targets_;
    Eigen::Vector3d loudestTarget_;
    double loudestTargetPower_ = 0;
    const double targetDecay_ = 0.9;
    std::thread targetThread_;
    WaraPSClient targetClient_ = WaraPSClient("lk_target", WARAPS_ADDRESS, std::getenv("MQTT_USERNAME"),
                                              std::getenv("MQTT_PASSWORD"));
    std::chrono::duration<double> targetUpdateInterval_ = std::chrono::milliseconds(200);
    KalmanFilter3D kf_{0.4};

    gps_data_t *gpsData_;

    std::vector<std::reference_wrapper<AWProcessingUnit> > awpus_;
    std::vector<Eigen::Vector3d> awpu_positions_;

    double minProbability_ = 0.5;

    std::thread workerThread_;
    std::mutex mutex_;
    std::shared_ptr<bool> running = std::make_shared<bool>(false);

    // Mysig funktionssignatur, precis lagom lång
    static void FindTargetsRecursively(
        const std::vector<CartesianTarget> &toCompare,
        std::vector<std::vector<CartesianTarget> >::iterator begin,
        std::vector<std::vector<CartesianTarget> >::iterator end,
        std::vector<Eigen::Vector3d> &out
    );

public:
    explicit TargetHandler(gps_data_t *gpsData) : gpsData_(gpsData) {
    }

    ~TargetHandler();

    void Start();

    void Stop();

    void SetSensitivity(double sensitivity);

    std::vector<Eigen::Vector3d> getTargets();

    TargetHandler &operator<<(AWProcessingUnit &awpu);

    void FindTargets(std::vector<std::vector<Target> > &targets);

    void DisplayTarget(bool toggle);

    Eigen::Vector3d getLoudestTarget();

    // Neeej cpp är inte ett wordy språk alls
};


#endif //TARGETHANDLER_H
