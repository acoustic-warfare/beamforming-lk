/** @file pso_seeker.h
 * @author Irreq
 * @brief Particle Swarm Optimization (PSO) algorithm, used in antenna direction optimization.
 * The PSO algorithm is used to optimize the direction of antennas by iteratively improving solutions.
*/

#if 0
#ifndef BEAMFORMER_PSO_SEEKER_H
#define BEAMFORMER_PSO_SEEKER_H

#include <atomic>
#include <vector>

#include "../antenna.h"
#include "../config.h"
#include "../delay.h"
#include "../geometry.h"
#include "../pipeline.h"
#include "../streams.hpp"
#include "../worker.h"
#if USE_KALMAN_FILTER
#include "../kf.h"
#endif

#include <Eigen/Dense>


#define DIMENSIONS 2

/**
 * @class Particle
 * @brief A single particle in the PSO algorithm which is a potential solution.
 * The particle updates its position and velocity based on its own experience and that of its neighbors.
 */
class Particle {
public:
    /// Current direction of the particle.
    Spherical direction_current;
    /// Best known direction of the particle.
    Spherical direction_best;
    /// Current velocity of the particle.
    Spherical velocity;
    /// Best magnitude (fitness value) found by the particle.
    float best_magnitude;

    Antenna &antenna;
    Streams *streams;
    int id;

    //float (*objective_function)(float, float);

    //Particle() = delete;
    //Particle(const Particle &) = delete;
    //Particle(Particle &&) = default;

    /**
     * @brief Constructs a new Particle object.
     * @param antenna Reference to the associated antenna.
     * @param streams Pointer to the streams used for signal processing.
     * @param id Identifier of the particle.
     */
    Particle(Antenna &antenna, Streams *streams, int id);

    /**
     * @brief Sets a random direction to the current particle
     */
    void random();

    /**
     * @brief Computes the fitness of the particle based on given direction angles and number of sensors.
     * @param theta Direction angle theta.
     * @param phi Direction angle phi.
     * @param n_sensors Number of sensors.
     * @return The computed fitness value.
     */
    float compute(double theta, double phi, int n_sensors);

    /**
     * @brief Computes the fitness of the particle based on its current direction.
     * @return The computed fitness value.
     */
    float compute();

    /**
     * @brief Updates the particle's state, checking if a new best position has been found.
     */
    void update();
};


//void pso_finder(Pipeline *pipeline, int stream_id, int n_arrays);
#include <mutex>
/**
 * @class PSOWorker
 * @brief Manages a swarm of particles.
 */
class PSOWorker : public Worker {
public:
    /**
     * @brief Constructs a new PSOWorker object.
     * @param pipeline Pointer to the pipeline used for data streaming.
     * @param antenna Reference to the antenna being optimized.
     * @param running Pointer to a boolean flag indicating whether the worker is running.
     * @param swarm_size Number of particles in the swarm.
     * @param iterations Number of iterations for the PSO algorithm.
     */
    PSOWorker(Pipeline *pipeline, Antenna &antenna, bool *running, std::size_t swarm_size, std::size_t iterations);

    /**
     * @brief Main loop for the PSO worker, repeatedly updates particle states until stopped.
     */
    void loop();

    /**
     * @brief Initializes the particles in the swarm with random positions and velocities.
     */
    void initialize_particles();

    /**
     * @brief Draws a heatmap representing the particle positions.
     * @param heatmap Pointer to the heatmap matrix to be drawn.
     */
    void draw_heatmap(cv::Mat *heatmap);

    /**
     * @brief Sets the number of iterations for the PSO algorithm.
     * @param iterations Number of iterations.
     */
    void set_iterations(std::size_t iterations) {
        this->iterations = iterations;
    };

    /**
     * @brief Sets the number of particles in the swarm.
     * @param swarm_size Number of particles.
     */
    void set_swarm_size(std::size_t swarm_size) {
        this->swarm_size = swarm_size;
    };

    /**
     * @brief Gets the current number of iterations for the PSO algorithm.
     * @return The number of iterations.
     */
    std::size_t get_iterations() { return this->iterations; };

    /**
     * @brief Gets the current number of particles in the swarm.
     * @return The number of particles.
     */
    std::size_t get_swarm_size() { return this->swarm_size; };

    //Spherical getDirection();

private:
    /// Mutex for thread-safe operations in PSOWorker.
    std::mutex pso_lock;

    /// Number of particles in the swarm.
    std::size_t swarm_size;
    /// Number of iterations for the PSO algorithm.
    std::size_t iterations;

    /// Vector of particles in the swarm.
    std::vector<Particle> particles;
    /// Reference to the associated antenna.
    Antenna antenna;

    ///Spherical global_best_direction;
    float global_best_magnitude;
    /// Weight for the current velocity.
    double current_velocity_weight = 0.25;
    /// Weight for the new velocity.
    double new_velocity_weight = 1.0;
    /// Step size for updating the particles' positions.
    double delta = 1.0;
};

#endif //BEAMFORMER_PSO_SEEKER_H
#endif