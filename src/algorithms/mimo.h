#ifndef MIMO_H
#define MIMO_H

#include <Eigen/Dense>
#include <atomic>

#include "../worker.h"
#include "../antenna.h"
#include "../config.h"
#include "../pipeline.h"
#include "../streams.hpp"
#include "../delay.h"

class MIMOWorker : public Worker {
public:
    MIMOWorker(Pipeline *pipeline, Antenna &antenna, bool *running, int rows, int columns, float fov);
    ~MIMOWorker() {};
    worker_t get_type() {
        return worker_t::MIMO;
    };
protected:
    void reset() {};
    void update();
    void populateHeatmap(cv::Mat *heatmap);
    void setup();
    
private:
    int index = 0;
    int maxIndex;
    const int columns;
    const int rows;
    const float fov;
    float prevPower = 1.0;
    void computeDelayLUT();
    std::vector<std::vector<int>> offsetDelays;
    std::vector<std::vector<float>> fractionalDelays;
    std::vector<float> powerdB;
};

#endif