#include "gradient_ascend.h"

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
    const float norm = 1 / (float) antenna.usable;

    for (int n = 0; n < 4; n++) {

        Eigen::VectorXf tmp_delays = steering_vector_spherical(antenna, directionNearby[n].theta, directionNearby[n].phi);
        int i = 0;
        for (float del: tmp_delays) {
            
            double _offset;
            float fraction;
            fraction = (float) modf((double) del, &_offset);
            int offset = N_SAMPLES - (int) _offset;
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

        power[n] /= (float) N_SAMPLES;
    }

    float thetaPower = fmax(fabs(power[NORTH]), fabs(power[SOUTH]));
    float phiPower = fmax(fabs(power[EAST]), fabs(power[WEST]));

    directionGradient.theta = (double) ((power[SOUTH] - power[NORTH]) / thetaPower);/// (2.0 * delta));
    directionGradient.phi = (double) ((power[EAST] - power[WEST]) / phiPower);/// (2.0 * delta));
    magnitude = (power[NORTH] + power[EAST] + power[SOUTH] + power[WEST]) / 4.0;
    gradient = fabs(directionGradient.theta) + fabs(directionGradient.phi);
}


SphericalGradient::SphericalGradient(Pipeline *pipeline, Antenna &antenna, bool *running, std::size_t swarm_size, std::size_t iterations) : Worker(pipeline, running), antenna(antenna), swarm_size(swarm_size), iterations(iterations) {
    this->streams = pipeline->getStreams();
    thread_loop = std::thread(&SphericalGradient::loop, this);
}

void SphericalGradient::initialize_particles() {
    
    particles.clear();
    for (int i = 0; i < swarm_size; i++) {
        particles.emplace_back(this->antenna, this->streams);
        GradientParticle &particle = particles.back();
        particle.delta = TO_RADIANS(10.0);//1.0 + drandom() * 1.0);
    }
}

/**
 * Draw heatmap
 */
void SphericalGradient::draw_heatmap(cv::Mat *heatmap) {
    double x_res = (double) heatmap->rows;
    double y_res = (double) heatmap->cols;

    // Reset heatmap
    heatmap->setTo(cv::Scalar(0));

    lock.lock();

#if 0
    for (GradientParticle &particle: currentTrackers) {
        if (!particle.tracking) {
            continue;
        }
        // If we convert to sphere with radius 0.5 we don't have to normalize it other than add
        // 0.5 to get the sphere in the first sector in the cartesian coordinate system
        Cartesian position = Cartesian::convert(particle.directionCurrent, 1.0);

        // Use the position values to plot over the heatmap
        int x = (int) (x_res * (position.x / 2.0 + 0.5));
        int y = (int) (y_res * (position.y / 2.0 + 0.5));

        float gradient = 1.0 - clip(particle.gradient, 0.0, 2.0) / 2.0;

        heatmap->at<uchar>(y, x) = (255);

        int m = 255;//(int) clip(particle.magnitude / 1e-6, 0.0, 255.0);

        cv::Mat &frame = *heatmap;

        cv::circle(frame, cv::Point(x, y), 1 + (int) (gradient * 10.0), cv::Scalar(m, m, m), cv::FILLED, 8, 0);


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

        heatmap->at<uchar>(x,y) = (255);

        float mag = particle.magnitude; //20 * std::log10(particle.magnitude * 1e5);

        int m = (int) (mag / maxValue * 255.0);

        cv::circle(*heatmap, cv::Point(y, x), 1 + (int)(gradient * 10.0), cv::Scalar(m,m,m), cv::FILLED, 8, 0);


        heatmap->at<uchar>(x, y) = 255;
    }
#endif

    lock.unlock();
}

void SphericalGradient::loop() {
    std::cout << "Starting loop" << std::endl;
    double learning_rate;
    constexpr double start_rate = 1e-1;
    // Place particles on dome
    int it = 0;
    initialize_particles();

    int n_trackers = 9;
    
    for (int i = 0; i < n_trackers; i++) {
        currentTrackers.emplace_back(this->antenna, this->streams);
        GradientParticle& particle = currentTrackers.back();
        particle.delta = TO_RADIANS(3);
    }

    while (looping && pipeline->isRunning()) {

        // Wait for incoming data
        pipeline->barrier();

        lock.lock();

        if (it % 100 == 0) {
            initialize_particles();
        }
        it++;
        


        int start = this->pipeline->mostRecent();

        for (int i = 0; i < iterations; i++) {
            // This loop may run until new data has been produced, meaning its up to
            // the machine to run as fast as possible
            if (this->pipeline->mostRecent() != start) {
                //std::cout << "Too slow: " << i + 1 << "/" << iterations << std::endl;
                break;// Too slow!
            }
            int n_tracking = 0;
            for (auto &particle : currentTrackers) {
                if (particle.tracking) {
                    for (int i = 0; i < 5; i++) 
                        particle.step(start_rate / 100.0);
                    if (particle.gradient > 1.0) {
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

                for (int n = m+1; n < n_trackers; n++) {
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
                    if (particle.gradient < 1e-4) {
                        for (auto &tracker : currentTrackers) {
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
        for (auto &tracker : currentTrackers) {
            if (tracker.tracking) {
                tracking.emplace_back(tracker.directionCurrent, tracker.magnitude, 1 / tracker.gradient);
            }
        }
        lock.unlock();
    }

    std::cout << "Done loop" << std::endl;
}