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
    Spherical directionCurrent;
    Spherical directionNearby[4];
    Spherical directionGradient;
    Spherical directionBest;
    float magnitude;
    Antenna &antenna;
    Streams *streams;

    int offset_delays[4][ELEMENTS];
    float fractional_delays[4][ELEMENTS];

    float epsilon;
    double delta;

    GradientParticle(Antenna &antenna, Streams *streams);

    void nearby();
    void jump();

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
    SphericalGradient(Pipeline *pipeline, Antenna &antenna, bool *running, std::size_t swarm_size, std::size_t iterations);


    void loop();
    void draw_heatmap(cv::Mat *heatmap);

    void initialize_particles();

    worker_t get_type() {
        return worker_t::GRADIENT;
    };

protected:
    //Spherical sphericalGradient(const Spherical &spherical, const double delta = 1e-6) {
    //    float measurments[4];
    //    measurments[0] = compute_direction(streams, antenna, );
    //    double grad_theta =
    //}

private: 
    std::size_t iterations;
    std::size_t swarm_size;
    Streams *streams;
    Antenna antenna;
    std::vector<GradientParticle> particles;

};



#endif

