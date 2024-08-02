/** @file target_handler.h
 * @author Janne
 * @brief Class definition for handling targets in the WARA PS system.
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
 * @class TargetHandler
 * @brief Class that classifies and draws targets to the WARA PS display.
 * Takes two or more active AWPUS as inputs and classifies the targets provided.
 */
class TargetHandler {
public:
    /**
     * @brief Constructor for the TargetHandler class.
     * @param gpsData Pointer to GPS data structure.
     * @param lk_heading Initial heading of the system in degrees.
     */
    explicit TargetHandler(gps_data_t *gpsData, const double lk_heading = 0) : lk_heading_(lk_heading * std::numbers::pi / 180),
                                                                               gpsData_(gpsData) {
    }

    TargetHandler(const TargetHandler &) = delete;
    TargetHandler operator=(const TargetHandler &) = delete;

    /**
     * @brief Destructor, cleanups of resources.
     */
    ~TargetHandler();

    /**
     * @brief Spools up the target handler and worker thread. Does not block the current thread.
     */
    void Start();

    /**
     * @brief Gracefully stops the target thread.
     */
    void Stop();

    /**
     * @brief Sets the minimum required target gradient.
     * @param sensitivity Minimum target gradient to be valid for interferemetry.
     */
    void SetSensitivity(double sensitivity);

    /**
     * @brief Retrieves the list of current targets.
     * @return Vector of target positions.
     */
    std::vector<Eigen::Vector3d> getTargets();

    /**
     * @brief Add an AWPU worker to triangulate from.
     * @param awpu AWPU worker.
     * @param position The physical position of the microphone array in relation to ljudkriget (origin).
     * @return Self reference for cool chaining actions.
     */
    TargetHandler &AddAWPU(AWProcessingUnit *awpu, const Eigen::Vector3d &position);

    /**
     * @brief Set the target display on the WARA PS display.
     * @param toggle Display state.
     */
    void DisplayToWaraPS(bool toggle);

    /**
     * @brief Get the current best target (i.e the current active track with the most hits).
     * @return The target position in relation to ljudkriget (origin).
     */
    Eigen::Vector3d getBestTarget() const;

protected:
    /**
     * @brief AWPU Target converted a line with origin in the AWPU position.
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
    const double lk_heading_;
    gps_data_t *gpsData_ = nullptr;
    std::chrono::duration<double> waraPSUpdateInterval_ = std::chrono::milliseconds(500);

    static constexpr bool debugLogging_ = false;
    std::ofstream logFile_;

    std::vector<AWProcessingUnit *> awpus_;
    std::vector<Eigen::Vector3d> awpu_positions_;

    // Tracker hit and timeout sensitivities
    static constexpr double kMinGradient_ = 1000,
                            kMinTrackHitDistance = 0.2,
                            kMaxTrackHitDistance = 1;

    static constexpr long kMinTrackTimeoutTime = static_cast<long>(0.5),
                          kMaxTrackTimeoutTime = 2;


    std::thread workerThread_;
    std::shared_ptr<bool> running = std::make_shared<bool>(false);

    /**
     * @brief Finds the intersecting targets from multiple AWPUs.
     * @param targets Vector of vectors containing targets from each AWPU.
     */
    void FindIntersects(std::vector<std::vector<Target>> &targets);

    /**
     * @brief Updates the current tracks based on the found targets.
     */
    void UpdateTracks();

    /**
     * @brief Checks the tracks for a given triangulated target.
     * @param target The triangulated target to check.
     */
    void CheckTracksForTarget(TriangulatedTarget &target);

    /**
     * @brief Calculates the distance threshold for a given track.
     * @param track The track for which to calculate the threshold.
     * @return The calculated distance threshold.
     */
    static double CalculateDistanceThreshold(const Track &track);

    /**
     * @brief Calculates the duration threshold for a given track.
     * @param track The track for which to calculate the threshold.
     * @return The calculated duration threshold.
     */
    static long CalculateDurationThreshold(const Track &track);

    /**
     * @brief Recursively finds intersecting targets.
     * @param toCompare The Cartesian targets to compare.
     * @param begin The beginning iterator of the Cartesian targets.
     * @param end The ending iterator of the Cartesian targets.
     * @param out Vector to store the resulting triangulated targets.
     */
    void FindIntersectsRecursively(std::vector<CartesianTarget> &toCompare,
                                   std::vector<std::vector<CartesianTarget>>::iterator begin,
                                   std::vector<std::vector<CartesianTarget>>::iterator end,
                                   std::vector<TriangulatedTarget> &out);
};

#endif//TARGETHANDLER_H
