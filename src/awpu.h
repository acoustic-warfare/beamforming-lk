/** @file */

#ifndef AWPU_H
#define AWPU_H

#include <algorithm>

#include "algorithms/pso_seeker.h"
#include "antenna.h"
#include "audio/audio_wrapper.h"
#include "pipeline.h"
#include "worker.h"

class AWProcessingUnit {
public:
    AWProcessingUnit(const char *address, const int port, int verbose = 1);
    ~AWProcessingUnit();

    bool start(const worker_t worker);
    bool stop(const worker_t worker);
    void pause();
    void resume();
    void draw_heatmap(cv::Mat *heatmap);
    void play_audio();
    void stop_audio();
    void calibrate(const float reference_power_level = 1e-5);
    Spherical target();

    Pipeline *pipeline;

protected:
    int verbose;
    std::vector<Worker *> workers;
    
    bool running;

    std::vector<Antenna> antennas;

    AudioWrapper *audioWrapper;
};

#endif