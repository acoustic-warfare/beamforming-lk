#include "gradient_ascend.h"

constexpr double start_rate = 1e-1;

GradientParticle::GradientParticle(Antenna &antenna, Streams *streams) : antenna(antenna), streams(streams) {
    this->epsilon = 1e-9f;
    this->delta = TO_RADIANS(7);
    this->tracking = false;
    random();
}

void GradientParticle::random() {
    directionCurrent.theta = drandom() * (M_PI / 2.0);
    directionCurrent.phi = drandom() * (2.0 * M_PI);
}

void GradientParticle::jump() {
    constexpr double step = TO_RADIANS(2);
    directionCurrent.theta += (drandom() * 2.0 -1.0) * step;
    directionCurrent.phi += (drandom() * 2.0 - 1.0) * step;
    directionCurrent.phi = wrapAngle(directionCurrent.phi);
    directionCurrent.theta = clip(directionCurrent.theta, 0.0, M_PI / 2.0);
}

void GradientParticle::step(double rate) {
    update();

    Spherical newDirection;

    newDirection.theta = directionCurrent.theta + rate * directionGradient.theta;
    newDirection.phi = directionCurrent.phi + (rate * directionGradient.phi) / sin(1e-9 + directionCurrent.theta);

    newDirection.phi = wrapAngle(newDirection.phi);
    newDirection.theta = clip(newDirection.theta, 0.0, M_PI / 2.0);

    directionCurrent = newDirection;
}

void GradientParticle::nearby() {
    std::vector<Spherical> near = directionCurrent.nearby(delta);
    for (int i = 0; i < 4; i++) {
        directionNearby[i] = near[i];
        directionNearby[i].phi = wrapAngle(directionNearby[i].phi);
        directionNearby[i].theta = clip(directionNearby[i].theta, 0.0, M_PI / 2.0);
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
    }

    float thetaPower = fmax(fabs(power[NORTH]), fabs(power[SOUTH]));
    float phiPower = fmax(fabs(power[EAST]), fabs(power[WEST]));

    directionGradient.theta = static_cast<double>((power[SOUTH] - power[NORTH]) / thetaPower);/// (2.0 * delta));
    directionGradient.phi = static_cast<double>((power[EAST] - power[WEST]) / phiPower);/// (2.0 * delta));
    magnitude = (power[NORTH] + power[EAST] + power[SOUTH] + power[WEST]) / 4.0;
    gradient = fabs(directionGradient.theta) + fabs(directionGradient.phi);
}


SphericalGradient::SphericalGradient(Pipeline *pipeline, Antenna &antenna, bool *running, std::size_t swarm_size, std::size_t iterations) : Worker(pipeline, antenna, running), swarm_size(swarm_size), iterations(iterations) {
    this->n_trackers = 9;

    for (int i = 0; i < n_trackers; i++) {
        currentTrackers.emplace_back(this->antenna, this->streams);
        GradientParticle &particle = currentTrackers.back();
        particle.delta = TO_RADIANS(5);
    }

    initialize_particles();
}

void SphericalGradient::initialize_particles() {
    
    particles.clear();
    for (int i = 0; i < swarm_size; i++) {
        particles.emplace_back(this->antenna, this->streams);
        GradientParticle &particle = particles.back();
        particle.delta = TO_RADIANS(15.0);//1.0 + drandom() * 1.0);
    }
}

/**
 * Draw heatmap
 */
void SphericalGradient::populateHeatmap(cv::Mat *heatmap) {
    double x_res = (double) heatmap->rows;
    double y_res = (double) heatmap->cols;

#if 1
    for (GradientParticle &particle: currentTrackers) {
        if (!particle.tracking) {
            continue;
        }
        // If we convert to sphere with radius 0.5 we don't have to normalize it other than add
        // 0.5 to get the sphere in the first sector in the cartesian coordinate system
        Cartesian position = Cartesian::convert(particle.directionCurrent, 1.0);

        // Use the position values to plot over the heatmap
        int x = static_cast<int>(x_res * (position.x / 2.0 + 0.5));
        int y = static_cast<int>(y_res * (position.y / 2.0 + 0.5));

        float gradient = 1.0 - clip(particle.gradient, 0.0, 2.0) / 2.0;

        heatmap->at<uchar>(y, x) = (255);

        int m = 255;//(int) clip(particle.magnitude / 1e-6, 0.0, 255.0);

        cv::Mat &frame = *heatmap;

        cv::circle(frame, cv::Point(x, y), 1 + static_cast<int>(gradient * 10.0), cv::Scalar(m, m, m), cv::FILLED, 8, 0);


        heatmap->at<uchar>(y, x) = 255;
    }
#else
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
        int x = (int) (x_res * (position.x / 2.0 + 0.5));
        int y = (int) (y_res * (position.y / 2.0 + 0.5));

        float gradient = 1.0 - clip(particle.gradient, 0.0, 2.0) / 2.0;

        //heatmap->at<uchar>(x,y) = (255);

        float mag = particle.magnitude; //20 * std::log10(particle.magnitude * 1e5);

        int m = 3; //(int) (mag / maxValue * 255.0);

        cv::circle(*heatmap, cv::Point(y, x), 1 + (int)(gradient * 30.0), cv::Scalar(m,m,m), cv::FILLED, 8, 0);


        //heatmap->at<uchar>(x, y) = 255;
    }
#endif
}

void SphericalGradient::reset() {
    if (resetCount++ % 32 == 0) {
        initialize_particles();
    }
}

void SphericalGradient::update() {
    while (canContinue()) {
        int n_tracking = 0;
        for (auto &particle: currentTrackers) {
            if (particle.tracking) {
                for (int i = 0; i < 5; i++)
                    particle.step(start_rate / 50.0);
                if (particle.gradient > 1000) {
                    particle.tracking = false;
                } else {
                    n_tracking++;
                }
            }
        }

        for (int m = 0; m < n_trackers; m++) {
            if (!currentTrackers[m].tracking) {
                continue;
            }

            for (int n = m + 1; n < n_trackers; n++) {
                if (!currentTrackers[n].tracking) {
                    continue;
                }
                if (currentTrackers[m].directionCurrent.angle(currentTrackers[n].directionCurrent) < M_PI / 20.0) {
                    currentTrackers[m].tracking = false;
                }
            }
        }
        for (auto &particle: particles) {
            particle.step(start_rate);
            if (n_tracking < n_trackers) {
                if (particle.gradient < 1e-2) {
                    for (auto &tracker: currentTrackers) {
                        if (!tracker.tracking && tracker.directionCurrent.angle(particle.directionCurrent) > M_PI / 20.0) {
                            tracker.tracking = true;
                            tracker.directionCurrent = particle.directionCurrent;
                        }
                    }
                }
            }
        }
    }

    tracking.clear();
    for (auto &tracker: currentTrackers) {
        if (tracker.tracking) {
            tracking.emplace_back(tracker.directionCurrent, tracker.magnitude, 1 / tracker.gradient);
        }
    }
}