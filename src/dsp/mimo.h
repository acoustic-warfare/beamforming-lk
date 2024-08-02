/** @file mimo.h
 * @author Irreq
 * @brief Handles MIMO (multiple input, multiple output) beamforming algorithms.
 * */

#ifndef MIMO_H
#define MIMO_H

#include <Eigen/Dense>
#include <atomic>

#include "antenna.h"
#include "delay.h"
#include "pipeline.h"
#include "worker.h"

#define USE_REFERENCE 0// If mimo should reference a single mic
#define USE_DB 0       // If power should be represented in decibel scale

/** 
 * @class MIMOWorker
 * @brief Handles the setup, updating, and heatmap population for MIMO processing.
*/
class MIMOWorker : public Worker {
public:
    /**
     * @brief Constructs a MIMOWorker object.
     * @param pipeline Pointer to the pipeline object, audio data.
     * @param antenna Reference to the antenna object.
     * @param running Pointer to the running flag.
     * @param rows Number of rows in the MIMO array.
     * @param columns Number of columns in the MIMO array.
     * @param fov Field of view for the MIMO processing, in degrees.
     */
    MIMOWorker(Pipeline *pipeline, Antenna &antenna, bool *running, int rows, int columns, float fov);

    /**
     * @brief Destructor, cleanups of resources.
     */
    ~MIMOWorker(){};

    /**
     * @brief Gets the type of the worker.
     * @return The worker type, which is MIMO.
     */
    worker_t get_type() {
        return worker_t::MIMO;
    };

protected:
    /**
     * @brief Resets the worker's state.
     */
    void reset() override{};

    /**
     * @brief Updates the worker's state.
     */
    void update() override;

    /**
     * @brief Populates the heatmap with the current state.
     * @param heatmap Pointer to the heatmap matrix.
     */
    void populateHeatmap(cv::Mat *heatmap) override;

    /**
     * @brief Sets up the worker before starting.
     */
    void setup() override{};

private:
    int index = 0;        ///< Current index for processing.
    int maxIndex;         ///< Maximum index value.
    const int columns;    ///< Number of columns in the MIMO array.
    const int rows;       ///< Number of rows in the MIMO array.
    const float fov;      ///< Field of view for the MIMO processing.
    float prevPower = 1.0;///< Previous power value for processing.

    /**
     * @brief Computes the delay lookup table for the MIMO processing.
     */
    void computeDelayLUT();

    /// Vector of offset delays.
    std::vector<std::vector<int>> offsetDelays;
    /// Vector of fractional delays.
    std::vector<std::vector<float>> fractionalDelays;
    /// Vector of power values in decibel scale.
    std::vector<float> powerdB;
};

#endif//MIMO_H