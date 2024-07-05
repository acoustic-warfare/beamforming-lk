#ifndef PSO_H
#define PSO_H

#include <atomic>
#include <vector>

#include "../antenna.h"
#include "../config.h"
#include "../delay.h"
#include "../pipeline.h"
#include "../streams.hpp"
#include "../worker.h"

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

    double current_velocity_weight = 0.25;// Wight for current velocity
    double new_velocity_weight = 2.0;
    double delta = 0.5;


#if USE_KALMAN_FILTER
    KalmanFilter3D kf;
#endif

    //float (*objective_function)(float, float);

    PSO(int n_particles, Antenna &antenna, Streams *streams, int n_sensors);

    void initialize_particles();

    void optimize(int iterations);

    Eigen::Vector3f sanitize();
};


//void pso_finder(Pipeline *pipeline, int stream_id, int n_arrays);
#include <mutex>
class PSOWorker : public Worker {
public:
    PSOWorker(Pipeline *pipeline, bool *running, std::size_t swarm_size, std::size_t iterations);

    //~PSOWorker();
    //worker_t get_type() {
    //    return worker_t::PSO;
    //}

    void loop();

    void initialize_particles();

    void draw_heatmap(cv::Mat *heatmap);


    void set_iterations(std::size_t iterations) {
        this->iterations = iterations;
    };

    void set_swarm_size(std::size_t swarm_size) {
        this->swarm_size = swarm_size;
    };

    std::size_t get_iterations() { return this->iterations; };

    std::size_t get_swarm_size() { return this->swarm_size; };

private:
    std::mutex pso_lock;
    std::size_t swarm_size;
    std::size_t iterations;
    std::vector<Particle> particles;
    Antenna antenna;
    double global_best_theta, global_best_phi;
    float global_best_magnitude;
    int n_particles;

    double current_velocity_weight = 0.25;// Wight for current velocity
    double new_velocity_weight = 2.0;
    double delta = 1.5;
};

#endif