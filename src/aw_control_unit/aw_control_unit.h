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
    AWControlUnit(bool use_waraps);

    /**
     * The MQTT Client does not have a move or copy constructor, this propagates all the way up to here were we can't support it either
     */
    AWControlUnit(const AWControlUnit&) = delete;

    AWControlUnit operator=(const AWControlUnit&) = delete;

    /**
     * @brief Spools up the AW Control unit and blocks the current thread until it finished.
     * @param ports List of ports to be used by the control unit.
     * @param ip_address IP address of the target device.
     * @param use_camera Flag indicating whether to use the camera.
     * @param camera The camera identifier (e.g., device path or ID).
     * @param audio_port Port number for audio communication.
     * @param mimo Flag indicating whether MIMO (Multiple Input Multiple Output) is enabled.
     * @param tracking Flag indicating whether tracking functionality is enabled.
     * @param mimo_res Resolution setting for MIMO.
     * @param verbose Flag indicating whether verbose logging is enabled.
     * @param record Flag indicating whether recording is enabled.
     * @param fov Field of view setting.
     * @param use_fps Flag indicating whether FPS (Frames Per Second) display is enabled.
     * @param use_logo Flag indicating whether to display a logo.
     * @param debug Flag indicating whether debug mode is enabled.
     * @param miso Flag indicating whether MISO (Multiple Input Multiple Output) is enabled
     */
    void Start(const std::vector<int>& ports, const std::string& ip_address, bool use_camera, const std::string& camera,
               int audio_port, bool mimo, bool tracking, int mimo_res, bool verbose, bool record, float fov, bool use_fps,
               bool use_logo, bool debug, bool miso);
};

#endif//AW_CONTROL_UNIT_H
