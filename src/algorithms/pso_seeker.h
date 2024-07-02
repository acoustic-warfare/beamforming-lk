#ifndef PSO_H
#define PSO_H

#include <atomic>
#include <vector>

#include "../antenna.h"
#include "../config.h"
#include "../delay.h"
#include "../pipeline.h"
#include "../streams.hpp"

#if USE_KALMAN_FILTER
#include "../kf.h"
#endif

#include <Eigen/Dense>

#define DIMENSIONS 2

class Particle {
public:
    double azimuth, elevation;
    double velocity_azimuth, velocity_elevation;
    double best_azimuth, best_elevation;
    float best_magnitude;

    Antenna &antenna;
    Streams *streams;

    //float (*objective_function)(float, float);

    Particle(Antenna &antenna, Streams *streams);

    void random();

    float compute(double azimuth, double elevation);

    void update();
};


class PSO {
public:
    std::vector<Particle> particles;
    double global_best_azimuth, global_best_elevation;
    float global_best_magnitude;
    int n_particles;
    Antenna &antenna;
    Streams *streams;

#if USE_KALMAN_FILTER
    KalmanFilter3D kf;
#endif

    //float (*objective_function)(float, float);

    PSO(int n_particles, Antenna &antenna, Streams *streams);

    void initialize_particles();

    void optimize(int iterations);

    Eigen::Vector3f sanitize();
};


void pso_finder(Pipeline *pipeline);

#endif