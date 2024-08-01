/** @file gradient_ascend.h
 * @author Irreq
 * @brief TODO:
*/

#ifndef GRADIENT_H
#define GRADIENT_H

#include <chrono>

#include "antenna.h"
#include "delay.h"
#include "geometry.h"
#include "pipeline.h"
#include "worker.h"
#include "particle.h"

/**
 * @brief Hyperparams for gradient ascent
 */
#define SEEKER_RESET_COUNTER 128       // Number of iterations before jump
#define SEEKER_SPREAD TO_RADIANS(7)   // Angle fov for seeker
#define TRACKER_STEPS 5                // Number of individual steps for tracker
#define TRACKER_SLOWDOWN 0.1             // How much slower tracker steps
#define TRACKER_CLOSENESS TO_RADIANS(4)// Angle between trackers before they are absorbed
#define TRACKER_ERROR_THRESHOLD 5e-2   // Error before tracker dies
#define TRACKER_MAX 5                  // Number of trackers
#define TRACKER_SPREAD TO_RADIANS(2)   // Angle fov for tracker
#define PARTICLE_RATE 5e-1              // Stepsize for particles

#define DEBUG_GRADIENT 1
#define MONOPULSE_DIRECTIONS 4// Quadrants
#define USE_HORIZONTAL 0      // Horizontal or quadrant monopulse

struct GradientParticle : public Particle {

    /**
     * Spread between the Monopulse angles
     */
    double spread;

    /**
     * Monopulse spread of directions with origin at `directionCurrent`
     * 
     *             NORTH
     * WEST  directionCurrent  EAST
     *             SOUTH
     */
    Spherical directionNearby[MONOPULSE_DIRECTIONS];


    /**
     * The absolute error for the angular gradient with zero being perfect
     */
    float gradientError = 0.0;

    GradientParticle(Antenna &antenna, Streams *streams, double fov, double spread);

    /**
     * Find nearby angles for monopulse directions
     */
    void findNearby();

    /**
     * Step in the current direction using the result from the monopulse response as 
     * guidance
     */
    void step(const double rate);
};

struct GradientSeeker : public GradientParticle {
    /**
     * If the seeker has jumped
     */
    bool jumped = false;

    GradientSeeker(Antenna &antenna, Streams *streams, double fov, double spread = TO_RADIANS(SEEKER_SPREAD));

    /**
     * Jump to another position in the search domain relative to the current position
     */
    void jump();
};

struct GradientTracker : public GradientParticle {
    /**
     * Mode of particle
     */
    bool tracking = false;

    /**
     * When particle first started tracking something
     */
    std::chrono::time_point<std::chrono::high_resolution_clock> start;

    GradientTracker(Antenna &antenna, Streams *streams, double fov, double spread = TO_RADIANS(TRACKER_SPREAD));

    /**
     * Allow the tracker to follow in a certain direction
     */
    void startTracking(Spherical direction);

    /**
     * Stop tracking process
     */
    void stopTracking();

    /**
     * Overload greater than
     */
    bool operator>(const GradientTracker &other) const;
};


/**
 * @class SphericalGradient
 * @brief Worker for finding sources using spherical Gradient descent
 */
class SphericalGradient : public Worker {
public:
    SphericalGradient(Pipeline *pipeline, Antenna &antenna, bool *running, std::size_t swarm_size, std::size_t iterations, float fov);

    worker_t get_type() {
        return worker_t::GRADIENT;
    };

    int seekerResetCounter = SEEKER_RESET_COUNTER;
    double seekerSpread = SEEKER_SPREAD;
    int trackerSteps = TRACKER_STEPS;
    double trackerSlowdown = TRACKER_SLOWDOWN;
    double trackerCloseness = TRACKER_CLOSENESS;
    double trackerErrorThreshold = TRACKER_ERROR_THRESHOLD;
    int trackerMax = TRACKER_MAX;
    double trackerSpread = TRACKER_SPREAD;

protected:
    void update() override;
    void reset() override;
    void populateHeatmap(cv::Mat *heatmap) override;

private:
    double mean = 0.0;
    float fov;
    int resetCount = 0;
    std::size_t n_trackers;
    std::size_t iterations;
    std::size_t swarm_size;
    std::vector<GradientSeeker> seekers;
    std::vector<GradientTracker> trackers;

    void initialize_particles();
};


#endif
