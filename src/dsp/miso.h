/** @file miso.h
 * @author Irreq
 * @brief MISO sound in a specific direction
*/

#ifndef MISO_H
#define MISO_H

#include "worker.h"
#include "gradient_ascend.h"
#include "geometry.h"
#include "audio_wrapper.h"

#define DEBUG_MISO 1

class MISOWorker : public Worker {
public:
    MISOWorker(Pipeline *pipeline, Antenna &antenna, bool *running, double fov);
    ~MISOWorker() {};
    worker_t get_type() { return worker_t::MISO; };
    void steer(Spherical direction) override;

    void startTracking(Spherical direction);
    void stopTracking() { tracking = false; };

protected:
    void reset() override {};
    void update() override;
    void populateHeatmap(cv::Mat *heatmap) override;
    void setup() override {};

private:
    double fov;
    float data[N_SAMPLES];

    double power = 0.0;

    /// @brief Ratio for heatmap fov
    double ratio;

    /// @brief If tracking source
    bool tracking = false;

    /// @brief Tracker of source
    GradientTracker beamformer;
    AudioWrapper aw;
};



#endif