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
    double theta, phi;
    double velocity_theta, velocity_phi;
    double best_theta, best_phi;
    float best_magnitude;

    Antenna &antenna;
    Streams *streams;
    int n_sensors;

    //float (*objective_function)(float, float);

    Particle(Antenna &antenna, Streams *streams, int n_sensors);

    void random();

    float compute(double theta, double phi, int n_sensors);

    void update();
};


class PSO {
public:
    std::vector<Particle> particles;
    double global_best_theta, global_best_phi;
    float global_best_magnitude;
    int n_particles;
    Antenna &antenna;
    Streams *streams;
    int n_sensors;


#if USE_KALMAN_FILTER
    KalmanFilter3D kf;
#endif

    //float (*objective_function)(float, float);

    PSO(int n_particles, Antenna &antenna, Streams *streams, int n_sensors);

    void initialize_particles();

    void optimize(int iterations);

    Eigen::Vector3f sanitize();
};


void pso_finder(Pipeline *pipeline, int stream_id, int n_arrays);

#endif