/** @file particle.h
 * @author Irreq
 * @brief TODO:
*/

#ifndef ALGORITHMS_COMMON_H
#define ALGORITHMS_COMMON_H

#include "antenna.h"
#include "delay.h"
#include "geometry.h"
#include "pipeline.h"
#include "streams.hpp"
#include "worker.h"

/**
 * @brief Normalize a spherical direction with a given theta limit.
 * @param direction The spherical direction to normalize.
 * @param theta_limit The upper limit for the theta angle.
 */
inline void normalizeSpherical(Spherical &direction, double theta_limit) {
    direction.phi = wrapAngle(direction.phi);
    direction.theta = clip(direction.theta, 0.0, theta_limit);
}

/** 
 * @brief Represents a particle in the gradient descent algorithm.
 */
struct Particle {
    Spherical directionCurrent;      ///< Current direction the particle is facing.
    Spherical directionGradient;     ///< Gradient at the current direction.
    Antenna &antenna;                ///< Antenna used for steering and checking valid sensors.
    Streams *streams;                ///< Data streams.
    double thetaLimit;               ///< Field of view in theta.
    int offsetDelays[ELEMENTS];      ///< Delay offsets.
    float fractionalDelays[ELEMENTS];///< Fractional delays.

    /**
     * @brief Constructs a Particle object.
     * @param antenna Reference to the Antenna object.
     * @param streams The streams of audio data.
     * @param fov Field of view for the particle, in degrees.
     */
    Particle(Antenna &antenna, Streams *streams, double fov);

    /**
     * @brief Place the particle on a random position in the search domain.
     */
    void random();

    /**
     * @brief Jump to another position in the search domain relative to the current position.
     * @param jumpSize Size of the jump.
     */
    void jump(const double jumpSize);

    /**
     * @brief Step in the current spherical gradient direction
     * @param rate Step rate in the gradient direction.
     */
    void step(const double rate);

    /**
     * @brief Determine if particle is close to a certain direction.
     * @param direction The direction to compare against.
     * @param angle The angle threshold for closeness.
     * @return True if the particle is within the specified angle of the direction, false otherwise.
     */
    bool isClose(const Spherical &direction, const double angle);

    /**
     * @brief Determine if particle is close to a certain particle.
     * @param particle The particle to compare against.
     * @param angle The angle threshold for closeness.
     * @return True if the particle is within the specified angle of the other particle, false otherwise.
     */
    bool isClose(const Particle &particle, const double angle);

    /**
     * @brief Amplitude delay and sum adaptive beamforming.
     * @return The power of the beamformed signal.
     */
    double beam();

    /**
     * @brief Steer the particle beam to a specific direction.
     * @param direction The direction to steer the beam towards.
     */
    void steer(const Spherical &direction);
};

#endif//ALGORITHMS_COMMON_H