#ifndef GRADIENT_H
#define GRADIENT_H

#include "../antenna.h"
#include "../config.h"
#include "../delay.h"
#include "../geometry.h"
#include "../pipeline.h"
#include "../streams.hpp"
#include "../worker.h"


class GradientParticle {
public:
    bool tracking;
    Spherical directionCurrent;
    Spherical directionNearby[4];
    Spherical directionGradient;
    Spherical directionBest;
    float magnitude;
    float gradient;
    Antenna &antenna;
    Streams *streams;

    int offset_delays[4][ELEMENTS];
    float fractional_delays[4][ELEMENTS];

    float epsilon;
    double delta;

    GradientParticle(Antenna &antenna, Streams *streams);

    void nearby();
    void jump();
    void step(double rate);

    void random();
    void update();

private:
};


/**
 * delta = 1e-6
    grad_theta = (function(theta + delta, phi) - function(theta - delta, phi)) / (2 * delta)
    grad_phi = (function(theta, phi + delta) - function(theta, phi - delta)) / (2 * delta)
    return np.array([grad_theta, grad_phi])
 */
class SphericalGradient : public Worker {
public:
    SphericalGradient(Pipeline *pipeline, Antenna &antenna, bool *running, std::size_t swarm_size, std::size_t iterations, float fov);

    worker_t get_type() {
        return worker_t::GRADIENT;
    };

protected:
    void update()override;
    void reset()override;
    void populateHeatmap(cv::Mat *heatmap)override;

private:
    float fov;
    int resetCount = 0;
    std::size_t n_trackers;
    std::size_t iterations;
    std::size_t swarm_size;
    std::vector<GradientParticle> particles;
    std::vector<GradientParticle> currentTrackers;

    void initialize_particles();
};


#endif
