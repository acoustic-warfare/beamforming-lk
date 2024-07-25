/** @file */

#ifndef AWPU_H
#define AWPU_H

#include <algorithm>

#include "algorithms/gradient_ascend.h"
#include "algorithms/mimo.h"
#include "algorithms/pso_seeker.h"
#include "antenna.h"
#include "audio/audio_wrapper.h"
#include "pipeline.h"
#include "worker.h"

class AWProcessingUnit {
public:
    AWProcessingUnit(const char *address, const int port, int verbose = 1, bool debug = false);
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

    Pipeline *pipeline;

protected:
    int verbose;
    bool debug;
    std::vector<Worker *> workers;
    Pipeline *pipeline;
    bool running = false;
    Spherical spherical;

    std::vector<Antenna> antennas;

    AudioWrapper *audioWrapper;
};



#endif