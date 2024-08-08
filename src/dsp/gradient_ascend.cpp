/** @file gradient_ascend.cpp
 * @author Irreq
*/

#include "gradient_ascend.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

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

void GradientParticle::step(const double rate, const double reference) {
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

    gradientError = (abs(phi) + abs(theta)) / sum;

#if RELATIVE
    directionGradient.theta = theta / reference;
    directionGradient.phi = phi / reference;
#else
    directionGradient.theta = theta;
    directionGradient.phi = phi;
#endif

    directionGradient.radius = sum / 4;
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


SphericalGradient::SphericalGradient(Pipeline *pipeline, Antenna &antenna, bool *running, std::size_t swarm_size, std::size_t iterations, float fov) : Worker(pipeline, antenna, running), swarm_size(swarm_size), fov(fov) {

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
        seekers.push_back(GradientSeeker(this->antenna, this->streams, this->fov, seekerSpread));// + drandom() * TO_RADIANS(20)));
    }
}

std::string timePointToString(const std::chrono::time_point<std::chrono::high_resolution_clock> &tp, const int id) {
    // Capture the end time
    auto end = std::chrono::high_resolution_clock::now();

    // Create a time string stream
    std::stringstream ss;

    ss << "  :" << id << " ";

    // Calculate the duration
    std::chrono::duration<double> duration = end - tp;

    // Print the duration rounded to 2 decimal places
    ss << std::fixed << std::setprecision(2);
    ss << duration.count() << "s";

    return ss.str();
}

/**
 * Draw heatmap
 */
void SphericalGradient::populateHeatmap(cv::Mat *heatmap) {
    double x_res = (double) heatmap->rows;
    double y_res = (double) heatmap->cols;

    float ratio = (float) sin((double) fov);
    int n_trackers = 0;

    // Define font face
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;

    // Define font scale (size)
    double fontScale = 1;

    // Define font color (here it's white)
    cv::Scalar color(255, 255, 255);

    // Define thickness of the text
    int thickness = 2;
    int i = 0;
    double bestValue = 0.0;
    auto end = std::chrono::high_resolution_clock::now();
    bool hasBest = false;
    cv::Point best;
    for (GradientTracker &particle: trackers) {
        i++;
        if (!particle.tracking) {
            continue;
        }
        // If we convert to sphere with radius 0.5 we don't have to normalize it other than add
        // 0.5 to get the sphere in the first sector in the cartesian coordinate system
        Cartesian position = Cartesian::convert(particle.directionCurrent, 1.0);

        // Use the position values to plot over the heatmap
        int x = static_cast<int>(x_res * (position.x / ratio / 2.0 + 0.5));
        int y = static_cast<int>(y_res * (position.y / ratio / 2.0 + 0.5));

#if 0
        if (particle.directionGradient.radius > bestValue) {
            bestValue = particle.directionGradient.radius;
            hasBest = true;
            best = cv::Point(x, y_res - y - 1);
        }
#else
        if (particle.start < end) {
            bestValue = particle.directionGradient.radius;
            end = particle.start;
            hasBest = true;
            best = cv::Point(x, y_res - y - 1);
        }

#endif

        float gradientError = 1.0 - clip(particle.gradientError, 0.0, 2.0) / 2.0;
        gradientError = 1.0;

        int m = 255;

        cv::Mat &frame = *heatmap;

        // Define the size of the square
        int size = 20;

        // Define the color of the square (B, G, R)
        cv::Scalar color(255, 255, 255);

        // Define the thickness of the square edges
        int thickness = 2;

        // Draw the hollow square
        cv::rectangle(*heatmap, cv::Point(x - size, y_res - y - 1 - size), cv::Point(x + size, y_res - y - 1 + size), color, thickness);

        n_trackers++;

        // Draw the text onto the image
        cv::putText(*heatmap, timePointToString(particle.start, i), cv::Point(x, y_res - y - 1 + 10), fontFace, fontScale, color, thickness);
    }

    std::stringstream ss;

    ss << "Tracking: " << n_trackers;
    cv::putText(*heatmap, ss.str(), cv::Point(0, 0), fontFace, fontScale, color, thickness);
    if (hasBest) {
        constexpr int length = 1000;
        constexpr int extra = 20;

        kf.update(Eigen::Vector3f((float) best.x, (float) best.y, 1.0));
        Eigen::Vector3f p = kf.predict(0);
        // Draw the tracker
        cv::circle(*heatmap, cv::Point(p(X_INDEX), p(Y_INDEX)), 30, color, 2, 8, 0);
        cv::line(*heatmap, cv::Point(p(X_INDEX), p(Y_INDEX)), cv::Point(best.x, best.y), color, thickness);
        // Draw the vertical line of the cross
        cv::line(*heatmap, cv::Point(best.x, best.y - length), cv::Point(best.x, best.y - extra), color, thickness);
        cv::line(*heatmap, cv::Point(best.x, best.y + extra), cv::Point(best.x, best.y + length), color, thickness);
        // Draw the horizontal line of the cross
        cv::line(*heatmap, cv::Point(best.x - length, best.y), cv::Point(best.x - extra, best.y), color, thickness);
        cv::line(*heatmap, cv::Point(best.x + extra, best.y), cv::Point(best.x + length, best.y), color, thickness);

        std::stringstream ss;
        ss << std::scientific << std::setprecision(2);
        ss << "Mag: " << bestValue << "W";
        cv::putText(*heatmap, ss.str(), cv::Point(best.x + 30, best.y - 10), fontFace, fontScale, color, thickness);
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
    float tmp_reference = 0.0;

    if (canContinue()) {
        float *out = streams->get_signal(0, N_SAMPLES);
        for (int i = 1; i < N_SAMPLES - 1; i++) {
            float MA = out[i] * 0.5f - 0.25f * (out[i + 1] + out[i - 1]);
            tmp_reference += powf(MA, 2);
        }
    }


    tmp_reference /= static_cast<float>(N_SAMPLES - 2);
    double reference = static_cast<double>(tmp_reference);
    while (canContinue()) {

        double maxPower = 0.0;
        Spherical bestDirection;
        bool better = false;

        // Run trackers
        int n_tracking = 0;
        for (auto &tracker: trackers) {
            if (tracker.tracking) {
                for (int i = 0; i < trackerSteps; i++) {
                    tracker.step(PARTICLE_RATE * trackerSlowdown, reference);
                }
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
                        trackers[m].stopTracking();
                    } else {
                        trackers[n].stopTracking();
                    }
                }
            }
        }

        float tmpMean = 0.0;
        int validCount = 0;

        // Run seekers
        for (auto &seeker: seekers) {
            seeker.step(PARTICLE_RATE, reference);

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

            if (seeker.directionGradient.radius > maxPower && seeker.gradientError < trackerErrorThreshold) {
                maxPower = seeker.directionGradient.radius;
                bestDirection = seeker.directionCurrent;
                better = true;
            }
        }

        if (better) {
            if (n_tracking < trackerMax) {
                for (auto &tracker: trackers) {
                    if (!tracker.tracking) {
                        tracker.startTracking(bestDirection);
                    }
                }
            }
        }

        mean = tmpMean / static_cast<double>(validCount);
    }

    // Fill the tracking vector to the DSP chain
    tracking.clear();
    for (GradientTracker &tracker: trackers) {
        if (tracker.directionGradient.radius < mean || tracker.directionGradient.radius < reference || tracker.gradientError > trackerErrorThreshold) {
            tracker.stopTracking();
            continue;
        }
        if (tracker.tracking) {
            tracking.emplace_back(tracker.directionCurrent, (float) tracker.directionGradient.radius, (float) (1 / tracker.gradientError), tracker.start);
        }
    }
}