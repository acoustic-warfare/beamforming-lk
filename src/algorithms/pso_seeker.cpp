#include "pso_seeker.h"

#define VALID_SENSOR(i) (64 <= i) && (i < 128)


float beamform(Streams *streams, int *offset_delays, float *fractional_delays) {
    float out[N_SAMPLES] = {0.0f};
    unsigned n = 0;

    for (unsigned s = 0; s < N_SENSORS; s++) {
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


Particle::Particle(Antenna &antenna, Streams *streams) : antenna(antenna), streams(streams) {
    azimuth = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 2.0 * ANGLE_LIMIT;
    elevation = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 2.0 * ANGLE_LIMIT;
    velocity_azimuth = 0.0f;
    velocity_elevation = 0.0f;
    best_azimuth = azimuth;
    best_elevation = elevation;
    best_magnitude = compute(azimuth, elevation);
}


void Particle::random() {
    azimuth = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 2.0 * ANGLE_LIMIT;
    elevation = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 2.0 * ANGLE_LIMIT;
}

float Particle::compute(double azimuth, double elevation) {
    Eigen::VectorXf tmp_delays = steering_vector_horizontal(antenna, azimuth, elevation);
    float fractional_delays[N_SENSORS];
    int offset_delays[N_SENSORS];
    int i = 0;
    for (float del: tmp_delays) {
        double _offset;
        float fraction;
        fraction = (float) modf((double) del, &_offset);
        int offset = N_SAMPLES - (int) _offset;
        // cout << del << endl;
        fractional_delays[i] = fraction;
        offset_delays[i] = offset;
        i++;
    }
    return beamform(streams, &offset_delays[0], &fractional_delays[0]);
}

void Particle::update() {
    float magnitude = compute(azimuth, elevation);
    best_magnitude *= 0.99;
    if (magnitude > best_magnitude) {
        best_magnitude = magnitude;
        best_azimuth = azimuth;
        best_elevation = elevation;
    }
}


inline float clip(const double n, const double lower, const double upper) {
    return std::max(lower, std::min(n, upper));
}

inline double wrapAngle(const double angle) {
    return angle - TWO_PI * floor(angle / TWO_PI);
}


PSO::PSO(int n_particles, Antenna &antenna, Streams *streams) : antenna(antenna), streams(streams), kf(0.1) {
    this->n_particles = n_particles;
    global_best_magnitude = 0.0f;
    initialize_particles();
}

void PSO::initialize_particles() {
    particles.clear();
    for (int i = 0; i < n_particles; i++) {
        particles.emplace_back(antenna, streams);

        Particle &particle = particles.back();

        if (particle.best_magnitude > global_best_magnitude) {
            global_best_magnitude = particle.best_magnitude;
            global_best_azimuth = particle.best_azimuth;
            global_best_elevation = particle.best_elevation;
        }
    }
}

void PSO::optimize(int iterations) {
    double w = 0.5f, c1 = 2.0f, c2 = 2.0f;
    double amount = 1.5;
    double delta = TO_RADIANS(FOV / (double) iterations * 2.0) * amount;
    global_best_magnitude *= 0.9f;

    for (int i = 0; i < iterations; i++) {
        for (auto &particle: particles) {
            particle.velocity_azimuth = w * particle.velocity_azimuth + c1 * static_cast<double>(rand()) / RAND_MAX * (particle.best_azimuth - particle.azimuth) * LOCAL_AREA_RATIO + c2 * static_cast<double>(rand()) / RAND_MAX * (global_best_azimuth - particle.azimuth) * GLOBAL_AREA_RATIO;
            particle.velocity_elevation = w * particle.velocity_elevation + c1 * static_cast<double>(rand()) / RAND_MAX * (particle.best_elevation - particle.elevation) * LOCAL_AREA_RATIO + c2 * static_cast<double>(rand()) / RAND_MAX * (global_best_elevation - particle.elevation) * GLOBAL_AREA_RATIO;
            particle.azimuth += particle.velocity_azimuth * delta;
            particle.elevation += particle.velocity_elevation * delta;

            particle.azimuth = clip(particle.azimuth, -ANGLE_LIMIT, ANGLE_LIMIT);
            particle.elevation = clip(particle.elevation, -ANGLE_LIMIT, ANGLE_LIMIT);

            particle.update();

            if (particle.best_magnitude > global_best_magnitude) {
                global_best_magnitude = particle.best_magnitude;
                global_best_azimuth = particle.best_azimuth;
                global_best_elevation = particle.best_elevation;
            }
        }
    }
}

Eigen::Vector3f PSO::sanitize() {
    Eigen::Vector3f sample(global_best_azimuth, global_best_elevation, 0.0);

#if USE_KALMAN_FILTER
    kf.update(sample);
    return kf.getState();
#else
    return sample;
#endif
}


void pso_finder(Pipeline *pipeline) {
    Antenna antenna = create_antenna(Position(0, 0, 0), COLUMNS, ROWS, DISTANCE);
    Streams *streams = pipeline->getStreams();

    PSO pso(40, antenna, streams);

    int newData;

    int prevX = 0;
    int prevY = 0;

    while (pipeline->isRunning()) {

        // Wait for incoming data
        pipeline->barrier();

        // This loop may run until new data has been produced, meaning its up to
        // the machine to run as fast as possible
        newData = pipeline->mostRecent();

        pso.initialize_particles();

        pso.optimize(30);

        //Eigen::Vector3f sample = pso.sanitize();
        //float azimuth = sample(0);
        //float elevation = sample(1);
        //azimuth = clip(azimuth, -ANGLE_LIMIT, ANGLE_LIMIT);
        //elevation = clip(elevation, -ANGLE_LIMIT, ANGLE_LIMIT);

        pipeline->magnitudeHeatmap->setTo(cv::Scalar(0));

#if 1
        for (auto &particle: pso.particles) {
            double azimuth = particle.best_azimuth;
            double elevation = particle.best_elevation;
            int xi = (int) ((double) X_RES * (sin(azimuth) / 2.0 + 0.5));
            int yi = (int) ((double) Y_RES * (sin(elevation) / 2.0 + 0.5));

            pipeline->magnitudeHeatmap->at<uchar>(xi, yi) = (uchar) (255);
        }
#else

        Eigen::Vector3f res = pso.sanitize();

        double x = sin((double) res(0));
        double y = sin((double) res(1));

        int xi = (int) ((double) X_RES * (x / 2.0 + 0.5));
        int yi = (int) ((double) Y_RES * (y / 2.0 + 0.5));

        pipeline->magnitudeHeatmap->at<uchar>(xi, yi) = (uchar) (255);

#endif

        //prevX = xi;
        //prevY = yi;
        pipeline->canPlot = 1;
        //std::cout << "(" << x << ", " << y << ")" << std::endl;
        std::cout << "Theta: " << pso.global_best_azimuth << " Phi: " << pso.global_best_elevation << std::endl;
    }
}