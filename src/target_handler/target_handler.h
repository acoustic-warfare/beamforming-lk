/** @file target_handler.h
 * @author Janne
 * @brief TODO:
 * @date 2024-07-18
*/

#ifndef TARGETHANDLER_H
#define TARGETHANDLER_H

#include <gps.h>
#include <wara_ps_client.h>

#include <ranges>

#include "aw_processing_unit.h"
#include "kf.h"

#define WARAPS_NAME "ljudkriget"
#define WARAPS_ADDRESS "mqtts://broker.waraps.org:8883"
#define GPS_ADDRESS "localhost"
#define GPS_PORT 2947

/**
 * @author Janne Schyffert
 * @date 2024-07-30
 * @class TargetHandler
 * @brief Class that classifies and draws targets to the WARA PS display.
 * Takes two or more active AWPUS as inputs and classifies the targets provided.
 */
class TargetHandler {
public:
    explicit TargetHandler(gps_data_t *gpsData) : gpsData_(gpsData) {
    }

    TargetHandler(const TargetHandler &) = delete;

    TargetHandler operator=(const TargetHandler &) = delete;

    // Implicitly deleted move constructor and move assignment operator

    ~TargetHandler();

    /**
     * @brief Spools up the target handler and worker thread. Does not block the current thread
     */
    void Start();

    /**
     * @brief Gracefully stops the target thread.
     */
    void Stop();

    /**
     * @brief Sets the minimum required target gradient
     * @param sensitivity minimum target gradient to be valid for interferemetry
     */
    void SetSensitivity(double sensitivity);

    std::vector<Eigen::Vector3d> getTargets();

    /**
     * @brief Add an AWPU worker to triangulate from
     * @param awpu AWPU worker
     * @param position The physical position of the microphone array in relation to ljudkriget (origin)
     * @return Self reference for cool chaining actions
     */
    TargetHandler &AddAWPU(AWProcessingUnit *awpu, const Eigen::Vector3d &position);

    /**
     * @brief Set the target display on the WARA PS display
     * @param toggle bool
     */
    void DisplayToWaraPS(bool toggle);

    /**
     * @brief Get the current best target (i.e the current active track with the most hits)
     * @return The target position in relation to ljudkriget (origin)
     */
    Eigen::Vector3d getBestTarget() const;

protected:
    /**
     * @brief AWPU Target converted a line with origin in the AWPU position
     */
    struct CartesianTarget {
        Eigen::ParametrizedLine<double, 3> directionLine;
        double power;
        double gradient;
        std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
    };

    /**
    * @brief An intersected target
    */
    struct TriangulatedTarget {
        Eigen::Vector3d position;
        double powerAverage = 0;
        long timeAlive = 0;
    };

    /**
     * @brief Track struct for intersected target quality tracking
     */
    struct Track {
        Eigen::Vector3d position;
        std::chrono::time_point<std::chrono::steady_clock> timeLastHit;
        bool valid;
        int hits;
    };

    std::vector<Track> tracks_;
    Track bestTrack_{Eigen::Vector3d::Zero(), std::chrono::steady_clock::now(), false, 0};

    std::thread targetThread_;
    WaraPSClient targetClient_ = WaraPSClient("lk_target", WARAPS_ADDRESS,
                                              std::getenv("MQTT_USERNAME") == nullptr
                                                  ? ""
                                                  : std::getenv("MQTT_USERNAME"),
                                              std::getenv("MQTT_PASSWORD") == nullptr
                                                  ? ""
                                                  : std::getenv("MQTT_PASSWORD"));
    gps_data_t *gpsData_ = nullptr;
    std::chrono::duration<double> waraPSUpdateInterval_ = std::chrono::milliseconds(500);

    static constexpr bool debugLogging_ = false;
    std::ofstream logFile_;

    std::vector<AWProcessingUnit *> awpus_;
    std::vector<Eigen::Vector3d> awpu_positions_;

    // Tracker hit and timeout sensitivities
    constexpr double kMinGradient_ = 1000,
            kMinTrackTimeoutTime = 0.5,
            kMaxTrackTimeoutTime = 2,
            kMinTrackHitDistance = 0.2,
            kMaxTrackHitDistance = 1;

    std::thread workerThread_;
    std::shared_ptr<bool> running = std::make_shared<bool>(false);

    void FindIntersects(std::vector<std::vector<Target> > &targets);

    void UpdateTracks();

    void CheckTracksForTarget(TriangulatedTarget &target);

    inline double CalculateDistanceThreshold(Track &track);

    inline long CalculateDurationThreshold(Track &track);

    // Mysig funktionssignatur, precis lagom lång
    void FindIntersectsRecursively(
        std::vector<CartesianTarget> &toCompare,
        std::vector<std::vector<CartesianTarget> >::iterator begin,
        std::vector<std::vector<CartesianTarget> >::iterator end,
        std::vector<TriangulatedTarget> &out);
};


#endif//TARGETHANDLER_H
