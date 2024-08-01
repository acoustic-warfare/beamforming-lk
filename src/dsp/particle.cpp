/** @file particle.cpp
 * @author Irreq
 */

#include "particle.h"

Particle::Particle(Antenna &antenna, Streams *streams, double fov) : antenna(antenna), streams(streams), thetaLimit(fov) {
    random();
};

void Particle::random() {
    directionCurrent.theta = drandom() * (thetaLimit);
    directionCurrent.phi = drandom() * (2.0 * M_PI);
}

void Particle::jump(const double jumpSize) {
    directionCurrent.theta += (drandom() * 2.0 - 1.0) * jumpSize;
    directionCurrent.phi += (drandom() * 2.0 - 1.0) * jumpSize;
    normalizeSpherical(directionCurrent, thetaLimit);
}

void Particle::step(const double rate) {
    directionCurrent.theta = directionCurrent.theta + rate * directionGradient.theta;
    directionCurrent.phi = directionCurrent.phi + (rate * directionGradient.phi) / sin(EPSILON + directionCurrent.theta);

    normalizeSpherical(directionCurrent, thetaLimit);
}

bool Particle::isClose(const Spherical &direction, const double angle) {
    return (directionCurrent.angle(direction) < angle);
}

bool Particle::isClose(const Particle &particle, const double angle) {
    return directionCurrent.angle(particle.directionCurrent) < angle;
}

void Particle::steer(const Spherical &direction) {
    int i = 0;
    for (float delay_idx: steering_vector_spherical(antenna, direction)) {

        double _offset;
        float fraction;
        fraction = static_cast<float>(modf((double) delay_idx, &_offset));
        int offset = N_SAMPLES - static_cast<int>(_offset);
        fractionalDelays[i] = fraction;
        offsetDelays[i] = offset;
        i++;
    }
}

double Particle::beam() {
    const float norm = 1 / static_cast<float>(antenna.usable);
    

    float out[N_SAMPLES] = {0.0};

    for (unsigned s = 0; s < antenna.usable; s++) {

        int i = antenna.index[s];
        float fraction = fractionalDelays[i];

        int offset = offsetDelays[i];

        float *signal = streams->get_signal(i, offset);
        delay(&out[0], signal, fraction);
    }
    float power_accumulator = 0.0f;
    for (int i = 0; i < N_SAMPLES; i++) {
        power_accumulator += powf(out[i] * norm, 2);
    }

    power_accumulator /= static_cast<float>(N_SAMPLES);

    return static_cast<double>(power_accumulator);
}