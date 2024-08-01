/** @file mimo.h
 * @author Irreq
 * @brief TODO:
*/

#ifndef MIMO_H
#define MIMO_H

#include <Eigen/Dense>
#include <atomic>

#include "antenna.h"
#include "delay.h"
#include "pipeline.h"
#include "worker.h"

#define USE_REFERENCE 0// If mimo should reference a single mic
#define USE_DB 0       // Decibel scale

/** 
 * @class MIMOWorker
 * @brief TODO:
*/
class MIMOWorker : public Worker {
public:
    MIMOWorker(Pipeline *pipeline, Antenna &antenna, bool *running, int rows, int columns, float fov);
    ~MIMOWorker() {};
    worker_t get_type() {
        return worker_t::MIMO;
    };

protected:
    void reset() override {};
    void update() override;
    void populateHeatmap(cv::Mat *heatmap) override;
    void setup() override {};

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