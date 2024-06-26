//
// Created by janne on 2024-06-24.
//

#ifndef BEAMFORMER_OPTIONS_H
#define BEAMFORMER_OPTIONS_H

#include <atomic>
#include <stdexcept>
#include <string>
#include <utility>

/**
 * Multithreading-safe options for the beamforming application.
 * Should be readable by multiple threads and writeable by a single master thread
 */
class BeamformingOptions {
public:
    BeamformingOptions() = default;

    BeamformingOptions(BeamformingOptions &&) = delete;
    BeamformingOptions(const BeamformingOptions &) = delete;
    BeamformingOptions &operator=(const BeamformingOptions &) = delete;

    std::atomic_bool have_changed_ = false;

    // Antenna
    std::uint32_t columns_ = 8;
    std::uint32_t rows_ = 8;
    float distance_ = 0.02;

    // Beamforming
    std::uint32_t x_res_ = 64;
    std::uint32_t y_res_ = 64;
    float theta_ = 0.0;
    float phi_ = 0.0;

    // FPGA
    std::uint32_t arrays_ = 4;
    float sample_rate_ = 48828.0;
    std::uint32_t protocol_version_ = 2;
    std::uint32_t n_sensors_ = columns_ * rows_ * arrays_;

    // Camera
    float camera_fov_ = 80.0;
    bool camera_on_ = false;
    std::string camera_path_ = "/dev/video0";

    // Audio
    bool audio_on_ = false;

    // UDP
    std::string udp_address_ = "127.0.0.1";
    std::string udp_port_ = "21844";
    std::uint32_t network_taps_ = 32;
};

#endif//BEAMFORMER_OPTIONS_H
