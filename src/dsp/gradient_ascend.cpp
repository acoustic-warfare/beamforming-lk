/** @file gradient_ascend.cpp
 * @author Irreq
*/

#include "gradient_ascend.h"


/**
 * @brief Parent Gradient particle
 */
GradientParticle::GradientParticle(Antenna &antenna, Streams *streams, double fov, double spread) : Particle(antenna, streams, fov), spread(spread) {}

void GradientParticle::findNearby() {
#if USE_HORIZONTAL
    std::vector<Spherical> near = directionCurrent.nearby(spread);
#else
    std::vector<Spherical> near = directionCurrent.quadrant(spread);
#endif
    for (int i = 0; i < MONOPULSE_DIRECTIONS; i++) {
        directionNearby[i] = near[i];
        normalizeSpherical(directionNearby[i], thetaLimit);
    }
}

void GradientParticle::step(const double rate) {
    findNearby();
#if USE_HORIZONTAL
    float power[MONOPULSE_DIRECTIONS] = {0.0};
    const float norm = 1 / static_cast<float>(antenna.usable);

    for (int n = 0; n < MONOPULSE_DIRECTIONS; n++) {
        steer(directionNearby[n]);
        power[n] = beam();
    }

    float thetaPower = fmax(fabs(power[NORTH]), fabs(power[SOUTH]));
    float phiPower = fmax(fabs(power[EAST]), fabs(power[WEST]));

    directionGradient.theta = static_cast<double>((power[SOUTH] - power[NORTH]) / thetaPower);
    directionGradient.phi = static_cast<double>((power[EAST] - power[WEST]) / phiPower);
    directionGradient.radius = static_cast<double>((power[NORTH] + power[EAST] + power[SOUTH] + power[WEST]) / static_cast<float>(MONOPULSE_DIRECTIONS));

    gradientError = fabs(directionGradient.theta) + fabs(directionGradient.phi);
#else
    steer(directionNearby[0]);
    double q1 = beam();

    steer(directionNearby[1]);
    double q2 = beam();

    steer(directionNearby[2]);
    double q3 = beam();

    steer(directionNearby[3]);
    double q4 = beam();

    double sum = q1 + q2 + q3 + q4;

    double phi = (q1 + q4) - (q2 + q3);
    double theta = (q3 + q4) - (q1 + q2);

    directionGradient.theta = theta / sum;
    directionGradient.phi = phi / sum;

    directionGradient.radius = sum;
    gradientError = abs(directionGradient.theta) + abs(directionGradient.phi);
#endif

    Particle::step(rate);
}


/**
 * @brief Gradient seeker
 */
GradientSeeker::GradientSeeker(Antenna &antenna, Streams *streams, double fov, double spread) : GradientParticle(antenna, streams, fov, spread) {}

void GradientSeeker::jump() {
    Particle::jump(thetaLimit / 2);
    jumped = true;
}


/**
 * @brief Gradient Tracker
 */
GradientTracker::GradientTracker(Antenna &antenna, Streams *streams, double fov, double spread) : GradientParticle(antenna, streams, fov, spread) {}

void GradientTracker::startTracking(Spherical direction) {
    directionCurrent = direction;
    tracking = true;
    start = std::chrono::high_resolution_clock::now();
}

void GradientTracker::stopTracking() {
    tracking = false;
}

bool GradientTracker::operator>(const GradientTracker &other) const {
    return start > other.start;
}


SphericalGradient::SphericalGradient(Pipeline *pipeline, Antenna &antenna, bool *running, std::size_t swarm_size, std::size_t iterations, float fov) : Worker(pipeline, antenna, running), swarm_size(swarm_size), iterations(iterations), fov(fov) {

    this->fov = TO_RADIANS(fov / 2.0);

    for (int i = 0; i < trackerMax; i++) {
        trackers.push_back(GradientTracker(this->antenna, this->streams, this->fov, trackerSpread));
    }

    initialize_particles();
    thread_loop = std::thread(&SphericalGradient::loop, this);
}

void SphericalGradient::initialize_particles() {

    seekers.clear();
    for (int i = 0; i < swarm_size; i++) {
        seekers.push_back(GradientSeeker(this->antenna, this->streams, this->fov, seekerSpread + drandom() * TO_RADIANS(20)));
    }
}

/**
 * Draw heatmap
 */
void SphericalGradient::populateHeatmap(cv::Mat *heatmap) {
    double x_res = (double) heatmap->rows;
    double y_res = (double) heatmap->cols;

    float ratio = (float) sin((double) fov);
    for (GradientTracker &particle: trackers) {
        if (!particle.tracking) {
            continue;
        }
        // If we convert to sphere with radius 0.5 we don't have to normalize it other than add
        // 0.5 to get the sphere in the first sector in the cartesian coordinate system
        Cartesian position = Cartesian::convert(particle.directionCurrent, 1.0);

        // Use the position values to plot over the heatmap
        int x = static_cast<int>(x_res * (position.x / ratio / 2.0 + 0.5));
        int y = static_cast<int>(y_res * (position.y / ratio / 2.0 + 0.5));

        float gradientError = 1.0 - clip(particle.gradientError, 0.0, 2.0) / 2.0;

        int m = 255;

        cv::Mat &frame = *heatmap;

        cv::circle(*heatmap, cv::Point(x, y_res - y - 1), 1 + (int) (gradientError * 15.0), cv::Scalar(m, m, m), cv::FILLED, 8, 0);
    }

#if DEBUG_GRADIENT
    double maxValue = 0.0;
    for (GradientSeeker &particle: seekers) {
        if (particle.directionGradient.radius > maxValue) {
            maxValue = particle.directionGradient.radius;
        }
    }

    //maxValue = 20 * std::log10(maxValue * 1e5);
    for (GradientSeeker &particle: seekers) {
        // If we convert to sphere with radius 0.5 we don't have to normalize it other than add
        // 0.5 to get the sphere in the first sector in the cartesian coordinate system
        Cartesian position = Cartesian::convert(particle.directionCurrent, 1.0);

        // Use the position values to plot over the heatmap
        int x = (int) (x_res * (position.x / ratio / 2.0 + 0.5));
        int y = (int) (y_res * (position.y / ratio / 2.0 + 0.5));

        float gradientError = 1.0 - clip(particle.gradientError, 0.0, 2.0) / 2.0;

        float mag = particle.directionGradient.radius;

        int m;
        if (particle.jumped) {
            m = 100;
            particle.jumped = false;
        } else {
            m = 255;
        }

        cv::circle(*heatmap, cv::Point(x, y_res - y - 1), 1 + (int) (gradientError * 5.0), cv::Scalar(m, m, m), cv::FILLED, 8, 0);
    }
#endif
}

void SphericalGradient::reset() {
    if (resetCount++ % seekerResetCounter == 0) {
        initialize_particles();
    }
}

void SphericalGradient::update() {
   // double reference = 
    while (canContinue()) {

        // Run trackers
        int n_tracking = 0;
        for (auto &tracker: trackers) {
            if (tracker.tracking) {
                for (int i = 0; i < trackerSteps; i++)
                    tracker.step(PARTICLE_RATE * trackerSlowdown);
                //if (tracker.gradientError > TRACKER_ERROR_THRESHOLD) {
                //    tracker.tracking = false;
                //} else {
                //    n_tracking++;
                //}
                n_tracking++;
            }
        }

        // Stop trackers
        for (int m = 0; m < trackerMax; m++) {
            if (!trackers[m].tracking) {
                continue;
            }

            for (int n = m + 1; n < trackerMax; n++) {
                if (!trackers[n].tracking) {
                    continue;
                }

                if (trackers[m].isClose(trackers[n], trackerCloseness)) {
                    if (trackers[m] > trackers[n]) {
                        trackers[m].tracking = false;
                    }
                }
            }
        }

        float tmpMean = 0.0;
        int validCount = 0;

        // Run seekers
        for (auto &seeker: seekers) {
            seeker.step(PARTICLE_RATE);

            // Compare with trackers and jump if too close
            bool jumped = false;
            for (auto &tracker: tracking) {
                if (seeker.isClose(tracker.direction, trackerCloseness)) {
                    seeker.jump();
                    jumped = true;
                    break;
                }
            }

            if (jumped) {
                continue;
            }

            // Dispatch trackers if converged
            validCount++;
            tmpMean += seeker.directionGradient.radius;

            if (n_tracking < trackerMax) {
                if (seeker.gradientError < trackerErrorThreshold &&
                    seeker.directionGradient.radius > mean) {
                    for (auto &tracker: trackers) {
                        if (!tracker.tracking) {
                            tracker.startTracking(seeker.directionCurrent);
                            seeker.jump();
                        }
                    }
                }
            }
        }

        mean = tmpMean / static_cast<double>(validCount);
    }

    // Fill the tracking vector to the DSP chain
    tracking.clear();
    for (GradientTracker &tracker: trackers) {
        if (tracker.directionGradient.radius < mean * 0.5) {
            tracker.stopTracking();
            continue;
        }
        if (tracker.tracking) {
            tracking.emplace_back(tracker.directionCurrent, (float)tracker.directionGradient.radius, (float)(1 / tracker.gradientError), tracker.start);
        }
    }
}