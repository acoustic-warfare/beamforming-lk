/** @file aw_control_unit.h
 * @author Janne, Tuva
 * @brief Manages AW Processing Units (AWPU) and handles data publishing, GPS data, and WaraPS client communication.
 * @date 2024-07-04
 */

#ifndef AW_CONTROL_UNIT_H
#define AW_CONTROL_UNIT_H

#include <wara_ps_client.h>

#include <vector>

#include "../target_handler/target_handler.h"
#include "aw_processing_unit.h"

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

    /// Classifies and draws targets to the WARA PS display
    TargetHandler targetHandler_;

    /**
     * @brief Publishes data from the control unit.
     */
    void publishData();

public:
    /**
     * @brief Default constructor for AWControlUnit.
     */
    AWControlUnit();

    /**
     * The MQTT Client does not have a move or copy constructor, this propagates all the way up to here were we can't support it either
     */
    AWControlUnit(const AWControlUnit&) = delete;

    AWControlUnit operator=(const AWControlUnit&) = delete;

    /**
     * @brief Spools up the AW Control unit and blocks the current thread until it finished.
     * @param TODO:
     */
    void Start(const std::vector<int>& ports, const std::string& ip_address, bool use_camera, const std::string& camera,
               bool audio, bool mimo, bool tracking, int mimo_res, bool verbose, bool record, float fov);
};

#endif//AW_CONTROL_UNIT_H
