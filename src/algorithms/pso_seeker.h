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
    Spherical direction_current;
    Spherical direction_best;
    Spherical velocity;
    float best_magnitude;

    Antenna &antenna;
    Streams *streams;
    int id;

    //float (*objective_function)(float, float);

    //Particle() = delete;
    //Particle(const Particle &) = delete;
    //Particle(Particle &&) = default;
    Particle(Antenna &antenna, Streams *streams, int id);

    void random();

    float compute(double theta, double phi, int n_sensors);
    float compute();

    void update();
};


//void pso_finder(Pipeline *pipeline, int stream_id, int n_arrays);
#include <mutex>
class PSOWorker : public Worker {
public:
    PSOWorker(Pipeline *pipeline, Antenna &antenna, bool *running, std::size_t swarm_size, std::size_t iterations);

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
    Spherical global_best_direction;
    float global_best_magnitude;
    int n_particles;

    double current_velocity_weight = 0.25;// Wight for current velocity
    double new_velocity_weight = 1.0;
    double delta = 1.0;
};

#endif