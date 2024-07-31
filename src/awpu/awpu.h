/** @file awpu.h
 * @author Irreq, Tuva
 * @brief Audio Warefare Processing Unit (AWPU) interface for managing audio processing tasks.
 */

#ifndef BEAMFORMER_AWPU_H
#define BEAMFORMER_AWPU_H

#include <algorithm>
#include <optional>

#include "algorithms/gradient_ascend.h"
#include "algorithms/mimo.h"
#include "gradient_ascend.h"
#include "mimo.h"
#include "pso_seeker.h"
#include "antenna.h"
#include "audio_wrapper.h"
#include "pipeline.h"
#include "worker.h"

#define FOV 63.0
#define MIMO_SIZE 256

/**
 * @class AWProcessingUnit
 * @brief Manages processing tasks for incoming audio data, including heatmap generation, audio playback, and microphone calibration.
 */
class AWProcessingUnit {
public:
    /**
     * @brief Constructor for AWProcessingUnit.
     * @param address The IP address for the processing unit (192.168.1.75-78).
     * @param port The port for the processing unit (21875-78).
     * @param fov Field of view for the processing unit.
     * @param small_res Resolution for MIMO processing.
     * @param verbose For extra logging.
     */
    AWProcessingUnit(const char *address, const int port, float fov = FOV, int small_res = MIMO_SIZE, int verbose = 1, bool use_audio = false);

    /**
     * @brief Constructor for AWProcessingUnit with a pipeline.
     * @param pipeline The processing pipeline.
     * @param verbose For extra logging.
     */
    AWProcessingUnit(Pipeline *pipeline, int verbose = 1, bool use_audio = false);

    /**
     * @brief Destructor, cleanups of resources.
     */
    ~AWProcessingUnit();

    /**
     * @brief Finds the number of antennas and usable microphones
     */
    void setupAntennas();

    /**
     * @brief Starts the processing unit with the specified worker.
     * @param worker The worker to start.
     * @return True if the worker started successfully, false otherwise.
     */
    bool start(const worker_t worker);

    /**
     * @brief Stops the processing unit with the specified worker.
     * @param worker The worker to stop.
     * @return True if the worker stopped successfully, false otherwise.
     */
    bool stop(const worker_t worker);

    /**
     * @brief Pauses the processing unit.
     */
    void pause();

    /**
     * @brief Resumes the processing unit.
     */
    void resume();

    /**
     * @brief Draws the heatmap of the current processing state.
     * @param heatmap Pointer to the heatmap matrix.
     */
    void draw_heatmap(cv::Mat *heatmap) const;

    /**
     * @brief Plays the audio associated with the processing unit (FPGA).
     */
    void play_audio();

    /**
     * @brief Stops the audio playback and saves any audio files
     */
    void stop_audio();

    /**
     * @brief Calibrates the microphone data to a reference power level.
     * @param reference_power_level The reference power level for calibration.
     */
    void calibrate(const float reference_power_level = 1e-5);

    /**
     * @brief Performs synthetic calibration of the processing unit.
     */
    void synthetic_calibration();

    /**
     * @brief Retrieves the detected targets from the processing unit.
     * @return A vector of detected targets.
     */
    std::vector<Target> targets();

    /**
     * @brief Draws the compact and normal visualization of the processing state.
     * @param compact Pointer to the compact visualization matrix.
     * @param normal Pointer to the normal visualization matrix.
     */
    void draw(cv::Mat *compact, cv::Mat *normal) const;

protected:
    /// Field of view for the processing unit
    float fov;

    /// Resolution for MIMO processing
    int small_res;

    /// Verbose for extra logging
    int verbose;

    /// Vector of workers managing different processing tasks
    std::vector<Worker *> workers;

    /// Pipeline to the FPGA from the PC, used to retrieve input data for any processing
    Pipeline *pipeline;

    /// Flag if the processing unit is running
    bool running = false;

    /// Vector of antennas managed by the processing unit
    std::vector<Antenna> antennas;

    /// Optional audio wrapper for handling audio playback
    std::optional<AudioWrapper> audioWrapper;
};

#endif //BEAMFORMER_AWPU_H