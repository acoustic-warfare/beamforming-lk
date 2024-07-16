#include "gradient_ascend.h"

GradientParticle::GradientParticle(Antenna &antenna, Streams *streams) : antenna(antenna), streams(streams) {
    this->epsilon = 1e-9f;
    this->delta = TO_RADIANS(7);
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

void GradientParticle::nearby() {
#if 1
    std::vector<Spherical> near = directionCurrent.nearby(delta);
#else
    // North
    directionNearby[NORTH].theta = directionCurrent.theta + delta;
    directionNearby[NORTH].phi = directionCurrent.phi;

    // East
    directionNearby[EAST].theta = directionCurrent.theta;
    directionNearby[EAST].phi = directionCurrent.phi + delta;// / sin(epsilon + directionCurrent.theta);

    // South
    directionNearby[SOUTH].theta = directionCurrent.theta - delta;
    directionNearby[SOUTH].phi = directionCurrent.phi;

    // West
    directionNearby[WEST].theta = directionCurrent.theta;
    directionNearby[WEST].phi = directionCurrent.phi - delta;// / sin(epsilon + directionCurrent.theta);
#endif
    //for (Spherical& spherical : near) {
    //    spherical.phi = wrapAngle(spherical.phi);
    //    spherical.theta = clip(spherical.theta, 0.0, M_PI / 2.0);
    //    directionNearby[i] = 
    //}
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

            //std::cout << "Using index: " << i << std::endl;

            int offset = offset_delays[n][i];

            //std::cout << offset << std::endl;

            float *signal = streams->get_signal(i, offset);
            delay(&out[0], signal, fraction);
            //delay_corrected(&out[0], signal, fraction, antenna.power_correction_mask[s]);
        }

        for (int i = 0; i < N_SAMPLES; i++) {
            power[n] += powf(out[i] * norm, 2);
        }

        power[n] /= (float) N_SAMPLES;
    }


    float thetaPower = ((power[NORTH] + power[SOUTH]) / 2.0);
    float phiPower = ((power[NORTH] + power[SOUTH]) / 2.0);
    directionGradient.theta = (double) - ((power[NORTH] - power[SOUTH]) / thetaPower); /// (2.0 * delta));
    directionGradient.phi = (double) ((power[EAST] - power[WEST]) / phiPower);/// (2.0 * delta));
    magnitude = (power[NORTH] + power[EAST] + power[SOUTH] + power[WEST]) / 4.0;

    //std::cout << "theta=" << directionGradient.theta << " phi=" << directionGradient.phi<< std::endl;
}


SphericalGradient::SphericalGradient(Pipeline *pipeline, Antenna &antenna, bool *running, std::size_t swarm_size, std::size_t iterations) : Worker(pipeline, running), swarm_size(swarm_size), iterations(iterations) {
    this->antenna = antenna;
    this->streams = pipeline->getStreams();
    thread_loop = std::thread(&SphericalGradient::loop, this);
}

void SphericalGradient::initialize_particles() {
    
    particles.clear();
    for (int i = 0; i < swarm_size; i++) {
        particles.emplace_back(this->antenna, this->streams);
        //GradientParticle &particle = particles.back();
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

    for (GradientParticle &particle: particles) {
        // If we convert to sphere with radius 0.5 we don't have to normalize it other than add
        // 0.5 to get the sphere in the first sector in the cartesian coordinate system
        Cartesian position = Cartesian::convert(particle.directionCurrent, 1.0);

        // Use the position values to plot over the heatmap
        int x = (int) (x_res * (position.x / 2.0 + 0.5));
        int y = (int) (y_res * (position.y / 2.0 + 0.5));

        cv::circle(*heatmap, cv::Point(y, x), 3, cv::Scalar(255, 255, 255), cv::FILLED, 8, 0);


        heatmap->at<uchar>(x, y) = 255;
    }

    lock.unlock();
}

void SphericalGradient::loop() {
    std::cout << "Starting loop" << std::endl;
    double learning_rate;
    constexpr double start_rate = 5e-3;
    // Place particles on dome
    int it = 0;
    initialize_particles();
    while (looping && pipeline->isRunning()) {

        // Wait for incoming data
        pipeline->barrier();

        lock.lock();
        
//        if (it % 40 == 0) {
//            // Place particles on dome
//            std::cout << "Replacing" << std::endl;
//            for (auto &particle: particles) {
//                particle.jump();
//            }
//            //initialize_particles();
//        }
//
//        if (it % 500 == 0) {
//            // Place particles on dome
//            //std::cout << "Replacing" << std::endl;
//            //for (auto &particle: particles) {
//            //    particle.jump();
//            //}
//            initialize_particles();
//        }
//        it++;
        


        int start = this->pipeline->mostRecent();

        for (int i = 0; i < iterations; i++) {
            // This loop may run until new data has been produced, meaning its up to
            // the machine to run as fast as possible
            if (this->pipeline->mostRecent() != start) {
                //std::cout << "Too slow: " << i + 1 << "/" << iterations << std::endl;
                break;// Too slow!
            }
            for (auto &particle: particles) {

                particle.update();
                

                Spherical newDirection;
                learning_rate = ((double)(iterations - i) / (double)iterations);
                particle.delta = TO_RADIANS(learning_rate * 10.0);

                learning_rate *= start_rate;
                newDirection.theta = particle.directionCurrent.theta + learning_rate * particle.directionGradient.theta;
                newDirection.phi = particle.directionCurrent.phi + (learning_rate * particle.directionGradient.phi)/ sin(1e-9+particle.directionCurrent.theta);

                newDirection.phi = wrapAngle(newDirection.phi);
                newDirection.theta = clip(newDirection.theta, 0.0, M_PI / 2.0);

                particle.directionCurrent = newDirection;
            }
        }

        lock.unlock();
    }

    std::cout << "Done loop" << std::endl;
}