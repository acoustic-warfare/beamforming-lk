/** @file gradient_ascend.h
 * @author Irreq
 * @brief TODO:
*/

#ifndef GRADIENT_H
#define GRADIENT_H

#include <chrono>

#include "../antenna.h"
#include "../config.h"
#include "../delay.h"
#include "../geometry.h"
#include "../pipeline.h"
#include "../streams.hpp"
#include "../worker.h"

/**
 * @class GradientParticle
 * @brief TODO:
 */
class GradientParticle {
public:
    bool jumped = false;
    bool tracking;
    Spherical directionCurrent;
    Spherical directionNearby[4];
    Spherical directionGradient;
    Spherical directionBest;
    float magnitude;
    float gradient;
    Antenna &antenna;
    Streams *streams;

    // Monopulse require comparing 4 items
    int offset_delays[4][ELEMENTS];
    float fractional_delays[4][ELEMENTS];

    float epsilon;
    double delta;

    std::chrono::time_point<std::chrono::high_resolution_clock> start;

    GradientParticle(Antenna &antenna, Streams *streams);

    void nearby();
    void jump();
    void step(double rate);

    void random();
    void update();

    void track(Spherical direction);
    bool isClose(const GradientParticle &other, const double angle);
    bool isClose(const Spherical &direction, const double angle);

    bool operator>(const GradientParticle &other) const {
        return start > other.start;
    }

private:
};


/**
 * Worker for finding sources using spherical Gradient descent
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
    float mean = 0.0;
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
