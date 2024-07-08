#ifndef AWPU_H
#define AWPU_H

#include "algorithms/pso_seeker.h"
#include "pipeline.h"
#include "worker.h"


class AWProcessingUnit {
public:
    AWProcessingUnit(const char *ip_address, const int port);
    ~AWProcessingUnit();

    bool start(const worker_t worker);
    bool stop(const worker_t worker);
    void pause();
    void resume();
    void draw_heatmap(cv::Mat *heatmap);


protected:
    //const char *ip_address;
    std::vector<Worker *> workers;
    Pipeline *pipeline;
    bool running;
};

#endif