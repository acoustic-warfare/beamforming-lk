/** @file aw_control_unit.h
 * @author Janne
 * @brief Manages AW Processing Units (AWPU) and handles data publishing, GPS data, and WaraPS client communication.
 * @date 2024-07-04
 */

#ifndef BEAMFORMER_AW_CONTROL_UNIT_H
#define BEAMFORMER_AW_CONTROL_UNIT_H

#include <wara_ps_client.h>

#include <condition_variable>
#include <vector>

#include "../TargetHandler/target_handler.h"
#include "awpu.h"

#define USE_AUDIO false
#define APPLICATION_NAME "Beamforming"
#define APPLICATION_WIDTH 1024
#define APPLICATION_HEIGHT 1024
#define X_RES 1024
#define Y_RES 1024
#define BLUR_KERNEL_SIZE 5


/**
 * @class AWControlUnit
 * @brief Manages AW Processing Units (AWPU) and handles data publishing, GPS data, and WaraPS client communication.
 */
class AWControlUnit {
private:
    /// WaraPS client instance
    WaraPSClient client_;
    /// Thread for running the WaraPS client
    std::thread data_thread_;
    /// Flag to indicate if WaraPS should be used
    bool usingWaraPS_ = false;

    /// GPS data structure
    gps_data_t gpsData_{};
    /// Flag to indicate if GPS should be used
    bool usingGps_ = false;

    /// Vector of AW Processing Units, to differentiate between APWUs (FPGAs)
    std::vector<AWProcessingUnit> awpus_;

    /// Classifies and draws targets to the WARA PS display
    TargetHandler targetHandler_;

    /// Condition variable to manage pausing of the control loop
    std::condition_variable pausedCV_{};
    /// Mutex to protect the pause condition variable
    std::mutex pauseMutex_;
    /// Flag to indicate if the unit is paused
    bool paused_ = false;

    /**
     * @brief Publishes data from the control unit.
     */
    void publishData();

public:
    /**
     * @brief Default constructor for AWControlUnit.
     */
    AWControlUnit();
    AWControlUnit(const AWControlUnit &) = delete;
    AWControlUnit operator=(const AWControlUnit &) = delete;

    /**
     * @brief Spools up the AW Control unit and blocks the current thread until it finished.
     */
    void Start();
};

#endif //BEAMFORMER_AW_CONTROL_UNIT_H
