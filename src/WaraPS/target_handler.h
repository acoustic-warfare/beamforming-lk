/** @file target_handler.h
 * @author Janne
 * @brief TODO:
 * @date 2024-07-18
*/

#ifndef TARGETHANDLER_H
#define TARGETHANDLER_H
#include <gps.h>
#include <ranges>
#include <wara_ps_client.h>

#include "../awpu.h"
#include "../kf.h"

/**
 * @author Janne Schyffert
 * @date 2024-07-30
 * @class TargetHandler
 * @brief Class that classifies and draws targets to the WARA PS display.
 * Class that classifies and draws targets to the WARA PS display.
 * Takes two or more active AWPUS as inputs and classifies the targets provided.
 */
class TargetHandler {
public:
    explicit TargetHandler(gps_data_t *gpsData) : gpsData_(gpsData) {}

    ~TargetHandler();

    /**
     * Spools up the target handler and worker thread. Does not block the current thread
     */
    void Start();

    /**
     * Gracefully stops the target thread.
     */
    void Stop();

    /**
     * Sets the minimum required target gradient
     * @param sensitivity minimum target gradient to be valid for interferemetry
     */
    void SetSensitivity(double sensitivity);

    std::vector<Eigen::Vector3d> getTargets();

    /**
     * Add an AWPU worker to triangulate from
     * @param awpu AWPU worker
     * @param position The physical position of the microphone array in relation to ljudkriget (origin)
     * @return Self reference for cool chaining actions
     */
     TargetHandler& AddAWPU(AWProcessingUnit *awpu, const Eigen::Vector3d& position);

    /**
     * Set the target display on the WARA PS display
     * @param toggle bool
     */
    void DisplayTarget(bool toggle);

    /**
     * Get the current best target (i.e the current active track with the most hits)
     * @return The target position in relation to ljudkriget (origin)
     */
    Eigen::Vector3d getBestTarget() const;

protected:
    /**
     * @brief TODO:
     */
    struct CartesianTarget {
        Eigen::ParametrizedLine<double, 3> directionLine;
        double power;
        double gradient;
        std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
    };

    /**
    * @brief TODO:
    */
    struct TriangulatedTarget {
        Eigen::Vector3d position;
        double powerAverage = 0;
        long timeAlive = 0;
    };

    struct Track {
        Eigen::Vector3d position;
        std::chrono::time_point<std::chrono::steady_clock> timeLastHit;
        bool valid;
        int hits;
    };

    std::vector<Track> tracks_;
    Track bestTrack_{Eigen::Vector3d::Zero(), std::chrono::steady_clock::now(), false, 0};


    // WARA PS Related variables
    std::thread targetThread_;
    WaraPSClient targetClient_ = WaraPSClient("lk_target", WARAPS_ADDRESS, std::getenv("MQTT_USERNAME"),
                                              std::getenv("MQTT_PASSWORD"));
    gps_data_t *gpsData_;
    std::chrono::duration<double> waraPSUpdateInterval_ = std::chrono::milliseconds(500);

    static constexpr bool debugLogging_ = false;
    std::ofstream logFile_;

    std::vector<AWProcessingUnit*> awpus_;
    std::vector<Eigen::Vector3d> awpu_positions_;

    double minGradient_ = 5;

    std::thread workerThread_;
    std::shared_ptr<bool> running = std::make_shared<bool>(false);

    void FindIntersects(std::vector<std::vector<Target> > &targets);

    void UpdateTracks();

    void CheckTracksForTarget(TriangulatedTarget &target);

    // Mysig funktionssignatur, precis lagom lång
     void FindIntersectsRecursively(
         std::vector<CartesianTarget> &toCompare,
         std::vector<std::vector<CartesianTarget>>::iterator begin,
         std::vector<std::vector<CartesianTarget>>::iterator end,
         std::vector<TriangulatedTarget> &out
     );
};


#endif //TARGETHANDLER_H
