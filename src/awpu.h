/** @file awpu.h
 * @author Irreq, Tuva
 * @brief TODO:
*/

#ifndef AWPU_H
#define AWPU_H

#include <algorithm>
#include <optional>

#include "algorithms/gradient_ascend.h"
#include "algorithms/mimo.h"
#include "algorithms/pso_seeker.h"
#include "antenna.h"
#include "audio/audio_wrapper.h"
#include "config.h"
#include "pipeline.h"
#include "worker.h"

/**
 * @class AWProcessingUnit
 * @brief TODO:
 */
class AWProcessingUnit {
public:
    AWProcessingUnit(const char *address, const int port, float fov = FOV, int small_res = MIMO_SIZE, int verbose = 1, bool debug = false);
    AWProcessingUnit(Pipeline *pipeline, int verbose = 1, bool debug = false);
    ~AWProcessingUnit();

    bool start(const worker_t worker);
    bool stop(const worker_t worker);
    void pause();
    void resume();
    void draw_heatmap(cv::Mat *heatmap) const;
    void play_audio();
    void stop_audio();
    void calibrate(const float reference_power_level = 1e-5);
    void synthetic_calibration();
    std::vector<Target> targets();

    void draw(cv::Mat *compact, cv::Mat *normal) const;

protected:
    int small_res;
    float fov;
    int verbose;
    bool debug;
    std::vector<Worker *> workers;
    Pipeline *pipeline;
    bool running = false;
    Spherical spherical;
    std::vector<Antenna> antennas;

    std::optional<AudioWrapper> audioWrapper;
};


#endif