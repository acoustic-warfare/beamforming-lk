#include "pso_seeker.h"

#define VALID_SENSOR(i) (64 <= i) && (i < 128)

#define USE_LUT 0

inline double drandom() {
    return static_cast<double>(rand()) / RAND_MAX;
}

#if USE_LUT

#define N_POINTS 1024

#define CHECK 90 * 360

float fractional_delays[CHECK][N_SENSORS];
int offset_delays[CHECK][N_SENSORS];

Eigen::MatrixXf dome;
Eigen::MatrixXi lookup_table(90, 360);

float beamform(Streams *streams, int index) {

#else
float beamform(Streams *streams, int *offset_delays, float *fractional_delays, int n_sensors) {
#endif
    float out[N_SAMPLES] = {0.0f};
    unsigned n = 0;

    for (unsigned s = 0; s < n_sensors; s++) {
#if 1
        if (VALID_SENSOR(s)) {
            float fraction = fractional_delays[s - 64];
            int offset = offset_delays[s - 64];

            //std::cout << "frac: " << fraction << " offset: " << offset << std::endl;

            float *signal = streams->get_signal(s, offset);
            delay(&out[0], signal, fraction);

            n++;
        }
#else
        float fraction = fractional_delays[s];
        int offset = offset_delays[s];

        //std::cout << "frac: " << fraction << " offset: " << offset << std::endl;

        float *signal = streams->get_signal(s, offset);
        delay(&out[0], signal, fraction);

        n++;

#endif
    }

    float power = 0.0f;
    float norm = 1 / (float) n;

    for (int i = 0; i < N_SAMPLES; i++) {
        power += powf(out[i] * norm, 2);
    }

    return power / (float) N_SAMPLES;
}


Particle::Particle(Antenna &antenna, Streams *streams, int n_sensors) : antenna(antenna), streams(streams), n_sensors(n_sensors) {
    random();
    velocity_theta = drandom() - 0.5;
    velocity_phi = drandom() - 0.5;
    best_theta = theta;
    best_phi = phi;
    best_magnitude = compute(theta, phi, n_sensors);
}


void Particle::random() {
    theta = drandom() * (2.0 * PI);
    phi = drandom() * PI_HALF;
}

float Particle::compute(double theta, double phi, int n_sensors) {
#if USE_LUT
    int itheta = (int) (phi * 180.0 / M_PI);
    int iphi = (int) (theta * 180.0 / M_PI);
    int index = lookup_table(90 - iphi, itheta);

    //return beamform(streams, index);

    //std::cout << "INdex: " << index << std::endl;

    if ((index < 0) || (index >= CHECK)) {
        std::cout << "WRONG: " << index << " theta: " << itheta << " phi: " << iphi << std::endl;
        return 0.0;
    } else {
        return beamform(streams, index);
    }

#else

    Eigen::VectorXf tmp_delays = steering_vector_spherical(antenna, theta, phi);

    float fractional_delays[ELEMENTS];
    int offset_delays[ELEMENTS];
    int i = 0;
    for (float del: tmp_delays) {
        double _offset;
        float fraction;
        fraction = (float) modf((double) del, &_offset);
        int offset = N_SAMPLES - (int) _offset;
        fractional_delays[i] = fraction;
        offset_delays[i] = offset;
        i++;
    }


    return beamform(streams, &offset_delays[0], &fractional_delays[0], n_sensors);

#endif
}

void Particle::update() {
    float magnitude = compute(theta, phi, n_sensors);
    //best_magnitude *= 0.99;
    if (magnitude > best_magnitude) {
        best_magnitude = magnitude;
        best_theta = theta;
        best_phi = phi;
    }
}


inline float clip(const double n, const double lower, const double upper) {
    return std::max(lower, std::min(n, upper));
}

inline double wrapAngle(const double angle) {
    return angle - TWO_PI * floor(angle / TWO_PI);
}


PSOWorker::PSOWorker(Pipeline *pipeline, bool *running, std::size_t swarm_size, std::size_t iterations) : Worker(pipeline, running), swarm_size(swarm_size), iterations(iterations) {
    this->global_best_magnitude = 0.0f;
    // Create antenna at origo
    this->antenna = create_antenna(Position(0, 0, 0), COLUMNS, ROWS, DISTANCE);

    //this->thread_loop = std::thread(this->loop);
    thread_loop = std::thread(&PSOWorker::loop, this);
}

worker_t Worker::get_type() {
    return worker_t::PSO;
}

void PSOWorker::initialize_particles() {
    particles.clear();
    for (int i = 0; i < swarm_size; i++) {
        particles.emplace_back(this->antenna, pipeline->getStreams(), pipeline->get_n_sensors());

        Particle &particle = particles.back();

        if (particle.best_magnitude > global_best_magnitude) {
            global_best_magnitude = particle.best_magnitude;
            global_best_theta = particle.best_theta;
            global_best_phi = particle.best_phi;
        }
    }
}


/**
 * Draw heatmap
 */
void PSOWorker::draw_heatmap(cv::Mat *heatmap) {
    double x_res = (double) heatmap->rows;
    double y_res = (double) heatmap->cols;

    // Reset heatmap
    heatmap->setTo(cv::Scalar(0));

    pso_lock.lock();

    for (Particle &particle: particles) {
        Position position = spherical_to_cartesian(particle.best_theta, particle.best_phi, 1.0);
        //Position position = spherical_to_cartesian(particle.best_theta, particle.best_phi, 0.5);
        //std::cout << "Pos: " << position << " x_res: " << x_res << std::endl;
        //heatmap->at<int>(
        //        (int) (x_res * (position(0) / 2.0 + 0.5)),
        //        (int) (y_res * (position(1) / 2.0 + 0.5))) = (uchar) 255;

        int xi = (int) (x_res * (position(0) / 2.0 + 0.5));
        int yi = (int) (y_res * (position(1) / 2.0 + 0.5));

        //int xi = (int) (x_res * (position(0) + 0.5));
        //int yi = (int) (y_res * (position(1) + 0.5));

        heatmap->at<int>(xi, yi) = (255);
    }

    pso_lock.unlock();
}


void PSOWorker::loop() {
    std::cout << "Starting loop" << std::endl;
    while (looping && pipeline->isRunning()) {
        // Wait for incoming data
        //std::cout << "Barrier wait" << std::endl;

        pipeline->barrier();
        //std::cout << "Barrier release" << std::endl;
        pso_lock.lock();
        // Place particles on dome
        initialize_particles();

        global_best_magnitude *= PSO_DECAY;
        global_best_magnitude = 0.0;

        
        int start = this->pipeline->mostRecent();
        
        for (int i = 0; i < iterations; i++) {
            // This loop may run until new data has been produced, meaning its up to
            // the machine to run as fast as possible
            if (this->pipeline->mostRecent() != start) {
                //std::cout << "Too slow: " << i + 1 << "/" << iterations << std::endl;
                //break;// Too slow!
            }
            for (auto &particle: particles) {

                // Compute new velocities based on distance to local best and global best
                particle.velocity_theta = current_velocity_weight * particle.velocity_theta + new_velocity_weight * (drandom() * (particle.best_theta - particle.theta) * LOCAL_AREA_RATIO + drandom() * (global_best_theta - particle.theta) * GLOBAL_AREA_RATIO);
                particle.velocity_phi = current_velocity_weight * particle.velocity_phi + new_velocity_weight * (drandom() * (particle.best_phi - particle.phi) * LOCAL_AREA_RATIO + drandom() * (global_best_phi - particle.phi) * GLOBAL_AREA_RATIO);

                // Move based on velocity
                particle.theta += particle.velocity_theta * delta;
                particle.phi += particle.velocity_phi * delta;

                // Limit the search area
                particle.theta = wrapAngle(particle.theta);
                particle.phi = clip(particle.phi, 0.0, PI_HALF);

                // Search using new positions
                particle.update();

                // Update global best
                if (particle.best_magnitude > global_best_magnitude) {
                    global_best_magnitude = particle.best_magnitude;
                    global_best_theta = particle.best_theta;
                    global_best_phi = particle.best_phi;
                }
            }


            //std::cout << "Best magnitude: " << global_best_magnitude << std::endl;
        }

        pso_lock.unlock();
        //std::cout << "Theta: " << global_best_theta << " Phi: " << global_best_phi << std::endl;
    }

    std::cout << "Done loop" << std::endl;
}
