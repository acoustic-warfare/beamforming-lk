#ifndef AWPU_H
#define AWPU_H

#include "algorithms/pso_seeker.h"
#include "pipeline.h"
#include "worker.h"


class AWProcessingUnit {
public:
    AWProcessingUnit(const char *address, const int port, int verbose);
    ~AWProcessingUnit();

    bool start(const worker_t worker);
    bool stop(const worker_t worker);
    void pause();
    void resume();
    void draw_heatmap(cv::Mat *heatmap);


protected:
    int verbose;
    std::vector<Worker *> workers;
    Pipeline *pipeline;
    bool running;
};

#endif