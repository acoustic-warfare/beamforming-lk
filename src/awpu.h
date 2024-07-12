/** @file */

#ifndef AWPU_H
#define AWPU_H

#include <algorithm>

#include "algorithms/pso_seeker.h"
#include "pipeline.h"
#include "worker.h"
#include "antenna.h"


class AWProcessingUnit {
public:
    AWProcessingUnit(const char *address, const int port, int verbose = 1);
    ~AWProcessingUnit();

    bool start(const worker_t worker);
    bool stop(const worker_t worker);
    void pause();
    void resume();
    void draw_heatmap(cv::Mat *heatmap);
    void calibrate(const float reference_power_level = 1e-5);


protected:
    int verbose;
    std::vector<Worker *> workers;
    Pipeline *pipeline;
    bool running;

    std::vector<Antenna> antennas;
};

#endif