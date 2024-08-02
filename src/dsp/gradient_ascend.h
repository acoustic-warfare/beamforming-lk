/** @file gradient_ascend.h
 * @author Irreq
 * @brief Handles gradient ascent beamforming.
 */

#ifndef GRADIENT_H
#define GRADIENT_H

#include <chrono>

#include "antenna.h"
#include "delay.h"
#include "geometry.h"
#include "particle.h"
#include "pipeline.h"
#include "worker.h"

// Hyperparams for gradient ascent
#define SEEKER_RESET_COUNTER 128       ///< Number of iterations before jump
#define SEEKER_SPREAD TO_RADIANS(7)    ///< Angle fov for seeker
#define TRACKER_STEPS 5                ///< Number of individual steps for tracker
#define TRACKER_SLOWDOWN 0.1           ///< How much slower tracker steps
#define TRACKER_CLOSENESS TO_RADIANS(4)///< Angle between trackers before they are absorbed
#define TRACKER_ERROR_THRESHOLD 5e-2   ///< Error before tracker dies
#define TRACKER_MAX 5                  ///< Number of trackers
#define TRACKER_SPREAD TO_RADIANS(2)   ///< Angle fov for tracker
#define PARTICLE_RATE 5e-1             ///< Stepsize for particles

#define DEBUG_GRADIENT 1
#define MONOPULSE_DIRECTIONS 4///< Quadrants
#define USE_HORIZONTAL 0      ///< Horizontal or quadrant monopulse

/**
 * @brief Particle used in the gradient ascent algorithm.
 */
struct GradientParticle : public Particle {
    /// Spread between the Monopulse angles
    double spread;

    /**
     * Monopulse spread of directions with origin at `directionCurrent`
     * 
     *             NORTH
     * WEST  directionCurrent  EAST
     *             SOUTH
     */
    Spherical directionNearby[MONOPULSE_DIRECTIONS];

    /// The absolute error for the angular gradient with zero being perfect
    float gradientError = 0.0;

    /**
     * @brief Constructs a GradientParticle.
     * @param antenna Reference to the antenna object.
     * @param streams The streams of audio data.
     * @param fov Field of view for the gradient particle, in degrees.
     * @param spread Spread angle for the gradient particle.
     */
    GradientParticle(Antenna &antenna, Streams *streams, double fov, double spread);

    /// Find nearby angles for monopulse directions
    void findNearby();

    /**
     * @brief Step in the current direction using the result from the monopulse response as 
     * guidance
     * @param rate The rate the step occurs.
     */
    void step(const double rate);
};


/** 
 * @brief A seeker particle for gradient ascent.
 */
struct GradientSeeker : public GradientParticle {
    ///True if the seeker has jumped
    bool jumped = false;

    /**
     * @brief Construct a new Gradient Seeker object.
     * @param antenna Reference to the antenna object.
     * @param streams The streams of audio data.
     * @param fov Field of view for the gradient seeker, in degrees.
     * @param spread Spread angle for the gradient seeker. 
     */
    GradientSeeker(Antenna &antenna, Streams *streams, double fov, double spread = TO_RADIANS(SEEKER_SPREAD));

    /**
     * @brief Jump to another position in the search domain relative to the current position
     */
    void jump();
};

/** 
 * @brief A tracker particle for gradient ascent.
 */
struct GradientTracker : public GradientParticle {
    /// Mode of particle
    bool tracking = false;

    /// When particle first started tracking something
    std::chrono::time_point<std::chrono::high_resolution_clock> start;

    /**
     * @brief Constructs a GradientTracker.
     * @param antenna Reference to the antenna object.
     * @param streams The streams of audio data.
     * @param fov Field of view for the gradient tracker, in degrees.
     * @param spread Spread angle for the gradient tracker.
     */
    GradientTracker(Antenna &antenna, Streams *streams, double fov, double spread = TO_RADIANS(TRACKER_SPREAD));

    /**
     * @brief Allow the tracker to follow in a certain direction
     * @param direction A spherical direction of the tracker.
     */
    void startTracking(Spherical direction);

    /**
     * @brief Stop tracking process.
     */
    void stopTracking();

    /**
     * @brief Overload greater than.
     */
    bool operator>(const GradientTracker &other) const;
};


/**
 * @class SphericalGradient
 * @brief Worker for finding sources using spherical Gradient descent.
 */
class SphericalGradient : public Worker {
public:
    /**
     * @brief Constructs a SphericalGradient worker.
     * @param pipeline Pointer to the Pipeline object, audio data.
     * @param antenna Reference to the Antenna object.
     * @param running Pointer to the running state flag.
     * @param swarm_size Number of seekers in the swarm.
     * @param iterations Number of iterations for the gradient descent process.
     * @param fov Field of view for the search area, in degrees.
     */
    SphericalGradient(Pipeline *pipeline, Antenna &antenna, bool *running, std::size_t swarm_size, std::size_t iterations, float fov);

    /**
     * @brief Gets the type of worker.
     * @return worker_t Type of worker (GRADIENT).
     */
    worker_t get_type() {
        return worker_t::GRADIENT;
    };

    int seekerResetCounter = SEEKER_RESET_COUNTER;         ///< Number of iterations before a seeker resets.
    double seekerSpread = SEEKER_SPREAD;                   ///< Spread angle for seekers.
    int trackerSteps = TRACKER_STEPS;                      ///< Number of steps for trackers.
    double trackerSlowdown = TRACKER_SLOWDOWN;             ///< Slowdown factor for tracker steps.
    double trackerCloseness = TRACKER_CLOSENESS;           ///< Closeness threshold for trackers.
    double trackerErrorThreshold = TRACKER_ERROR_THRESHOLD;///< Error threshold before a tracker stops.
    int trackerMax = TRACKER_MAX;                          ///< Maximum number of trackers.
    double trackerSpread = TRACKER_SPREAD;                 ///< Spread angle for trackers.

protected:
    /**
     * @brief Updates the state of the SphericalGradient worker.
     */
    void update() override;

    /**
     * @brief Resets the SphericalGradient worker.
     */
    void reset() override;

    /**
     * @brief Populates the heatmap with current data.
     * @param heatmap Pointer to the heatmap to be populated.
     */
    void populateHeatmap(cv::Mat *heatmap) override;

private:
    double mean = 0.0;                    ///< Mean value used in the gradient descent process.
    float fov;                            ///< Field of view for the search area.
    int resetCount = 0;                   ///< Counter for the number of resets.
    std::size_t n_trackers;               ///< Number of trackers.
    std::size_t iterations;               ///< Number of iterations for the gradient descent process.
    std::size_t swarm_size;               ///< Number of seekers in the swarm.
    std::vector<GradientSeeker> seekers;  ///< List of seekers.
    std::vector<GradientTracker> trackers;///< List of trackers.

    /**
     * @brief Initializes the particles (seekers and trackers).
     */
    void initialize_particles();
};

#endif//GRADIENT_H
