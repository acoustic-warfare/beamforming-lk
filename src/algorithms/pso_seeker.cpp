#include "pso_seeker.h"

#define VALID_SENSOR(i) (64 <= i) && (i < 128)

#define PSO_DECAY 0.6

#define SWARM_SIZE 100
#define SWARM_ITERATIONS

inline double drandom() {
    return static_cast<double>(rand()) / RAND_MAX;
}


float beamform(Streams *streams, int *offset_delays, float *fractional_delays, int n_sensors) {
    float out[N_SAMPLES] = {0.0f};
    unsigned n = 0;

    for (unsigned s = 0; s < n_sensors; s++) {
        if (VALID_SENSOR(s)) {
            float fraction = fractional_delays[s - 64];
            int offset = offset_delays[s - 64];

            float *signal = streams->get_signal(s, offset);
            delay(&out[0], signal, fraction);

            n++;
        }
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
    velocity_theta = 0.0f;
    velocity_phi = 0.0f;
    best_theta = theta;
    best_phi = phi;
    best_magnitude = compute(theta, phi, n_sensors);
}


void Particle::random() {
    theta = (drandom() - 0.5) * 2.0 * (2.0 * PI);
    phi = (drandom() - 0.5) * 2.0 * PI_HALF;
}

float Particle::compute(double theta, double phi, int n_sensors) {
    Eigen::VectorXf tmp_delays = steering_vector_spherical(antenna, theta, phi);
    //Eigen::VectorXf tmp_delays = steering_vector_horizontal(antenna, azimuth, elevation);
    float fractional_delays[n_sensors];
    int offset_delays[n_sensors];
    int i = 0;
    for (float del: tmp_delays) {
        //std::cout << "Del: " << del << std::endl;
        double _offset;
        float fraction;
        fraction = (float) modf((double) del, &_offset);
        int offset = N_SAMPLES - (int) _offset;
        // cout << del << endl;
        fractional_delays[i] = fraction;
        offset_delays[i] = offset;
        i++;
    }
    //std::cout << "azimuth: " << azimuth << " elevation: " << elevation << std::endl;
    return beamform(streams, &offset_delays[0], &fractional_delays[0], n_sensors);
}

void Particle::update() {
    float magnitude = compute(theta, phi, n_sensors);
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


PSO::PSO(int n_particles, Antenna &antenna, Streams *streams, int n_sensors) : antenna(antenna), streams(streams), kf(0.1) {
    this->n_particles = n_particles;
    this->n_sensors = n_sensors;
    global_best_magnitude = 0.0f;
    initialize_particles();
}

void PSO::initialize_particles() {
    particles.clear();
    for (int i = 0; i < n_particles; i++) {
        particles.emplace_back(antenna, streams, n_sensors);

        Particle &particle = particles.back();

        if (particle.best_magnitude > global_best_magnitude) {
            global_best_magnitude = particle.best_magnitude;
            global_best_theta = particle.best_theta;
            global_best_phi = particle.best_phi;
        }
    }
}


/**
 * Find maximum value and position using swarm search
 */
void PSO::optimize(int iterations) {

    global_best_magnitude *= PSO_DECAY;

    for (int i = 0; i < iterations; i++) {
        for (auto &particle: particles) {

            // Compute new velocities based on distance to local best and global best
            particle.velocity_theta = current_velocity_weight * particle.velocity_theta \
                                      + new_velocity_weight * (drandom() * (particle.best_theta - particle.theta) * LOCAL_AREA_RATIO + drandom() * (global_best_theta - particle.theta) * GLOBAL_AREA_RATIO);
            particle.velocity_phi = current_velocity_weight * particle.velocity_phi \
                                     + new_velocity_weight * (drandom() * (particle.best_phi - particle.phi) * LOCAL_AREA_RATIO + drandom() * (global_best_phi - particle.phi) * GLOBAL_AREA_RATIO);

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
    }
}

/**
 * Sanitize best global position by smoothing noise using Kalman filter
 */
Eigen::Vector3f PSO::sanitize() {
    Eigen::Vector3f sample(global_best_theta, global_best_phi, 0.0);

#if USE_KALMAN_FILTER
    kf.update(sample);
    return kf.getState();
#else
    return sample;
#endif
}


void pso_finder(Pipeline *pipeline, int stream_id, int n_sensors_in) {
    Antenna antenna = create_antenna(Position(0, 0, 0), COLUMNS, ROWS, DISTANCE);
    Streams *streams = pipeline->getStreams(stream_id);

    PSO pso(SWARM_SIZE, antenna, streams, n_sensors_in);

    int newData;

    int prevX = 0;
    int prevY = 0;

    while (pipeline->isRunning()) {

        // Wait for incoming data
        pipeline->barrier();

        // This loop may run until new data has been produced, meaning its up to
        // the machine to run as fast as possible
        newData = pipeline->mostRecent();

        // Place particles on dome
        pso.initialize_particles();

        // Optimize to find sources

        // Reset heatmap
        pipeline->magnitudeHeatmap->setTo(cv::Scalar(0));

#if 1
        for (auto &particle: pso.particles) {
            Position position = spherical_to_cartesian(particle.best_theta, particle.best_phi, 1.0);
            int xi = (int) ((double) X_RES * (position(0) / 2.0 + 0.5));
            int yi = (int) ((double) Y_RES * (position(1) / 2.0 + 0.5));

            pipeline->magnitudeHeatmap->at<uchar>(xi, yi) = (uchar) (255);
        }
#else

        Eigen::Vector3f res = pso.sanitize();

        Position position = spherical_to_cartesian(res(0), res(1), 1.0);

        int xi = (int) ((double) X_RES * (position(0) / 2.0 + 0.5));
        int yi = (int) ((double) Y_RES * (position(1) / 2.0 + 0.5));

        pipeline->magnitudeHeatmap->at<uchar>(xi, yi) = (uchar) (255);

#endif


        pipeline->canPlot = 1;
    }
}