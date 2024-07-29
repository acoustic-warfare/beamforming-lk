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
public:
    explicit TargetHandler(gps_data_t *gpsData) : gpsData_(gpsData) {}

    ~TargetHandler();

    void Start();

    void Stop();

    void SetSensitivity(double sensitivity);

    std::vector<Eigen::Vector3d> getTargets();

    TargetHandler& AddAWPU(AWProcessingUnit *awpu, const Eigen::Vector3d& position);

    void FindTargets(std::vector<std::vector<Target> > &targets);

    void DisplayTarget(bool toggle);

    Eigen::Vector3d getLoudestTarget() const;

protected:
    struct CartesianTarget {
        Eigen::ParametrizedLine<double, 3> directionLine;
        double power;
        double gradient;
        std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
    };

    struct TriangulatedTarget {
        Eigen::Vector3d position;
        double powerAverage;
        long timeAlive;
    };

    std::vector<TriangulatedTarget> targets_;
    TriangulatedTarget loudestTarget_{};
    const double targetDecay_ = 0.9;
    std::thread targetThread_;
    WaraPSClient targetClient_ = WaraPSClient("lk_target", WARAPS_ADDRESS, std::getenv("MQTT_USERNAME"),
                                              std::getenv("MQTT_PASSWORD"));
    std::chrono::duration<double> targetUpdateInterval_ = std::chrono::milliseconds(500);
    KalmanFilter3D kf_{static_cast<float>(targetUpdateInterval_.count() / 500)};

    gps_data_t *gpsData_;

    std::vector<AWProcessingUnit*> awpus_;
    std::vector<Eigen::Vector3d> awpu_positions_;

    double minGradient_ = 5;

    std::thread workerThread_;
    std::mutex mutex_;
    std::shared_ptr<bool> running = std::make_shared<bool>(false);

    static double CalculateTargetWeight(const TriangulatedTarget & triangulated_target);

    // Mysig funktionssignatur, precis lagom lång
     void FindTargetsRecursively(
         std::vector<CartesianTarget> &toCompare,
         std::vector<std::vector<CartesianTarget>>::iterator begin,
         std::vector<std::vector<CartesianTarget>>::iterator end,
         std::vector<TriangulatedTarget> &out
     );
};


#endif //TARGETHANDLER_H
