/** @file gradient_ascend.cpp
 * @author Irreq
*/

#include "gradient_ascend.h"

constexpr double start_rate = 1e-1;

float theta_limit = M_PI / 2;

GradientParticle::GradientParticle(Antenna &antenna, Streams *streams) : antenna(antenna), streams(streams) {
    this->epsilon = 1e-9f;
    this->delta = TO_RADIANS(7);
    this->tracking = false;
    random();
}

void GradientParticle::random() {
    directionCurrent.theta = drandom() * (theta_limit);
    directionCurrent.phi = drandom() * (2.0 * M_PI);
}

void GradientParticle::jump() {
    constexpr double step = TO_RADIANS(45);
    directionCurrent.theta += (drandom() * 2.0 -1.0) * step;
    directionCurrent.phi += (drandom() * 2.0 - 1.0) * step;
    directionCurrent.phi = wrapAngle(directionCurrent.phi);
    directionCurrent.theta = clip(directionCurrent.theta, 0.0, theta_limit);
    jumped = true;
}

void GradientParticle::step(double rate) {
    update();

    Spherical newDirection;

    newDirection.theta = directionCurrent.theta + rate * directionGradient.theta;
    newDirection.phi = directionCurrent.phi + (rate * directionGradient.phi) / sin(1e-9 + directionCurrent.theta);

    newDirection.phi = wrapAngle(newDirection.phi);
    newDirection.theta = clip(newDirection.theta, 0.0, theta_limit);

    directionCurrent = newDirection;
}

void GradientParticle::nearby() {
    std::vector<Spherical> near = directionCurrent.nearby(delta);
    for (int i = 0; i < 4; i++) {
        directionNearby[i] = near[i];
        directionNearby[i].phi = wrapAngle(directionNearby[i].phi);
        directionNearby[i].theta = clip(directionNearby[i].theta, 0.0, theta_limit);
    }
}

void GradientParticle::update() {
    nearby();

    float power[4] = {0.0};
    const float norm = 1 / static_cast<float>(antenna.usable);

    for (int n = 0; n < 4; n++) {

        Eigen::VectorXf tmp_delays = steering_vector_spherical(antenna, directionNearby[n].theta, directionNearby[n].phi);
        int i = 0;
        for (float del: tmp_delays) {
            
            double _offset;
            float fraction;
            fraction = static_cast<float>(modf((double) del, &_offset));
            int offset = N_SAMPLES - static_cast<int>(_offset);
            fractional_delays[n][i] = fraction;
            offset_delays[n][i] = offset;
            i++;
        }

        float out[N_SAMPLES] = {0.0};

        for (unsigned s = 0; s < antenna.usable; s++) {

            int i = antenna.index[s];
            float fraction = fractional_delays[n][i];

            int offset = offset_delays[n][i];

            float *signal = streams->get_signal(i, offset);
            delay(&out[0], signal, fraction);
            //delay_corrected(&out[0], signal, fraction, antenna.power_correction_mask[s]);
        }

        for (int i = 0; i < N_SAMPLES; i++) {
            power[n] += powf(out[i] * norm, 2);
        }

        power[n] /= static_cast<float>(N_SAMPLES);

        power[n] = powf(power[n], 3);
    }

    float thetaPower = fmax(fabs(power[NORTH]), fabs(power[SOUTH]));
    float phiPower = fmax(fabs(power[EAST]), fabs(power[WEST]));

    directionGradient.theta = static_cast<double>((power[SOUTH] - power[NORTH]) / thetaPower);/// (2.0 * delta));
    directionGradient.phi = static_cast<double>((power[EAST] - power[WEST]) / phiPower);/// (2.0 * delta));
    magnitude = (power[NORTH] + power[EAST] + power[SOUTH] + power[WEST]) / 4.0;
    gradient = fabs(directionGradient.theta) + fabs(directionGradient.phi);
}

void GradientParticle::track(Spherical direction) {
    directionCurrent = direction;
    tracking = true;
    start = std::chrono::high_resolution_clock::now();
}
bool GradientParticle::isClose(const GradientParticle &other, const double angle) {
    return (directionCurrent.angle(other.directionCurrent) < angle);
}

bool GradientParticle::isClose(const Spherical &direction, const double angle) {
    return (directionCurrent.angle(direction) < angle);
}


SphericalGradient::SphericalGradient(Pipeline *pipeline, Antenna &antenna, bool *running, std::size_t swarm_size, std::size_t iterations, float fov) : Worker(pipeline, antenna, running), swarm_size(swarm_size), iterations(iterations), fov(fov) {
    this->n_trackers = 5;

    this->fov = TO_RADIANS(fov/2.0);
    theta_limit = this->fov;

    for (int i = 0; i < n_trackers; i++) {
        currentTrackers.emplace_back(this->antenna, this->streams);
        GradientParticle &particle = currentTrackers.back();
        particle.delta = TO_RADIANS(4);
    }

    initialize_particles();
    thread_loop = std::thread(&SphericalGradient::loop, this);
}

void SphericalGradient::initialize_particles() {
    
    particles.clear();
    for (int i = 0; i < swarm_size; i++) {
        particles.emplace_back(this->antenna, this->streams);
        GradientParticle &particle = particles.back();
        particle.delta = TO_RADIANS(4);
    }
}

/**
 * Draw heatmap
 */
void SphericalGradient::populateHeatmap(cv::Mat *heatmap) {
    double x_res = (double) heatmap->rows;
    double y_res = (double) heatmap->cols;

    float ratio = (float) sin((double) fov);
    for (GradientParticle &particle: currentTrackers) {
        if (!particle.tracking) {
            continue;
        }
        // If we convert to sphere with radius 0.5 we don't have to normalize it other than add
        // 0.5 to get the sphere in the first sector in the cartesian coordinate system
        Cartesian position = Cartesian::convert(particle.directionCurrent, 1.0);
        
        // Use the position values to plot over the heatmap
        int x = static_cast<int>(x_res * (position.x/ratio / 2.0 + 0.5));
        int y = static_cast<int>(y_res * (position.y/ratio / 2.0 + 0.5));

        float gradient = 1.0 - clip(particle.gradient, 0.0, 2.0) / 2.0;

        heatmap->at<uchar>(y, x) = (255);

        int m = 255;

        cv::Mat &frame = *heatmap;

        cv::circle(*heatmap, cv::Point(x, y_res - y - 1), 1 + (int) (gradient * 15.0), cv::Scalar(m, m, m), cv::FILLED, 8, 0);
    }

    float maxValue = 0.0;
    for (GradientParticle &particle: particles) {
        if (particle.magnitude > maxValue) {
            maxValue = particle.magnitude;
        }
    }

    //maxValue = 20 * std::log10(maxValue * 1e5);
    for (GradientParticle &particle: particles) {
        // If we convert to sphere with radius 0.5 we don't have to normalize it other than add
        // 0.5 to get the sphere in the first sector in the cartesian coordinate system
        Cartesian position = Cartesian::convert(particle.directionCurrent, 1.0);

        // Use the position values to plot over the heatmap
        int x = (int) (x_res * (position.x/ratio / 2.0 + 0.5));
        int y = (int) (y_res * (position.y/ratio / 2.0 + 0.5));

        float gradient = 1.0 - clip(particle.gradient, 0.0, 2.0) / 2.0;

        float mag = particle.magnitude;

        int m;
        if (particle.jumped) {
            m = 100;
            particle.jumped = false;
        } else {
            m = 255;
        }

        cv::circle(*heatmap, cv::Point(x, y_res - y - 1), 1 + (int)(gradient * 5.0), cv::Scalar(m,m,m), cv::FILLED, 8, 0);
    }
}

void SphericalGradient::reset() {
    if (resetCount++ % 32 == 0) {
        initialize_particles();
    }
}

void SphericalGradient::update() {
    while (canContinue()) {
        
        // Run trackers
        int n_tracking = 0;
        for (auto &particle: currentTrackers) {
            if (particle.tracking) {
                for (int i = 0; i < 5; i++)
                    particle.step(start_rate / 50.0);
                if (particle.gradient > 1) {
                    particle.tracking = false;
                } else {
                    n_tracking++;
                }
            }
        }

        // Stop trackers
        for (int m = 0; m < n_trackers; m++) {
            if (!currentTrackers[m].tracking) {
                continue;
            }

            for (int n = m + 1; n < n_trackers; n++) {
                if (!currentTrackers[n].tracking) {
                    continue;
                }

                if (currentTrackers[m].isClose(currentTrackers[n], M_PI / 40.0)) {
                    if (currentTrackers[m] > currentTrackers[n]) {
                        currentTrackers[m].tracking = false;
                    }
                }
            }
        }

        float tmpMean = 0.0;
        int validCount = 0;

        // Run seekers
        for (auto &particle: particles) {
            particle.step(start_rate);

            // Compare with trackers and jump if too close
            bool jumped = false;
            for (auto &tracker : tracking) {
                if (particle.isClose(tracker.direction, M_PI / 40)) {
                    particle.jump();
                    jumped = true;
                    break;
                }
            }

            if (jumped) {
                continue;
            }

            // Dispatch trackers if converged
            validCount++;
            tmpMean += particle.magnitude;

            if (n_tracking < n_trackers) {
                if (particle.gradient < 1e-1 && particle.magnitude > mean) {
                    for (auto &tracker: currentTrackers) {
                        if (!tracker.tracking) {
                            tracker.track(particle.directionCurrent);
                            particle.jump();
                        }
                    }
                }
            }
        }

        mean = tmpMean / static_cast<float>(validCount);
    }

    // Fill the tracking vector to the DSP chain
    tracking.clear();
    for (auto &tracker: currentTrackers) {
        if (tracker.tracking) {
            tracking.emplace_back(tracker.directionCurrent, tracker.magnitude, 1 / tracker.gradient, tracker.start);
        }
    }
}