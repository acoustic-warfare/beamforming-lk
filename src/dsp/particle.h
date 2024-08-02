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

#define USE_BANDPASS 1 // If bandpass filter will be used

inline void normalizeSpherical(Spherical &direction, double theta_limit) {
    direction.phi = wrapAngle(direction.phi);
    direction.theta = clip(direction.theta, 0.0, theta_limit);
}

/** 
 * @brief TODO:
 */
struct Particle {
    /**
     * Current direction the particle is facing
     */
    Spherical directionCurrent;

    /**
     * Gradient (derivative) at the current direction. Not exactly spherical coordinates
     * but the angular gradient in two dimensions with radius being the mean power for all pulses
     */
    Spherical directionGradient;

    /**
     * Antenna used for steering and checking valid sensors
     */
    Antenna &antenna;

    /**
     * Data
     */
    Streams *streams;

    /**
     * Field of view in theta
     */
    double thetaLimit;

    /**
     * Delay variables
     */
    int offsetDelays[ELEMENTS];
    float fractionalDelays[ELEMENTS];

    Particle(Antenna &antenna, Streams *streams, double fov);

    /**
     * Place the particle on a random position in the search domain
     */
    void random();

    /**
     * Jump to another position in the search domain relative to the current position
     */
    void jump(const double jumpSize);

    /**
     * Step in the current spherical gradient direction
     */
    void step(const double rate);

    /**
     * Determine if particle is close to a certain direction
     */
    bool isClose(const Spherical &direction, const double angle);

    /**
     * Determine if particle is close to a certain particle
     */
    bool isClose(const Particle &particle, const double angle);

    /**
     * Amplitude delay and sum adaptive beamforming
     */
    double beam();

    /**
     * Steer the particle beam to a specific direction
     */
    void steer(const Spherical &direction);
};

#endif