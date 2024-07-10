#include "pso_seeker.h"

#if 0
#define VALID_SENSOR(i) (64 <= i) && (i < 128)
#else
#define VALID_SENSOR(i) (i < 64)
#endif


#define USE_LUT 0

inline double drandom() {
    return static_cast<double>(rand()) / RAND_MAX;
}


//#define IN_FIRST_SECTOR(i) ((i < 28) && )

#if USE_LUT

#define N_POINTS 1024

#define CHECK 90 * 360

float fractional_delays[CHECK][N_SENSORS];
int offset_delays[CHECK][N_SENSORS];

Eigen::MatrixXf dome;
Eigen::MatrixXi lookup_table(90, 360);

float beamform(Streams *streams, int index) {

#else
float beamform(Streams *streams, int *offset_delays, float *fractional_delays, int n_sensors, int id) {
#endif
    float out[N_SAMPLES] = {0.0f};
    unsigned n = 0;

    for (unsigned s = 0; s < n_sensors; s++) {
#if 1
        if (VALID_SENSOR(s)) {
#if 0
            int i = s - 64;

            const int *sector;

            switch (id) {
                case 0:
                    sector = &first_sector[0];
                    break;
                case 1:
                    sector = &second_sector[0];
                    break;
                case 2:
                    sector = &third_sector[0];
                    break;
                default:
                    sector = &fourth_sector[0];
                    break;
            }

            if (!in_sector(sector, i)) {
                continue;
            }
#endif

            int i = s % ELEMENTS;

            float fraction = fractional_delays[i];
            int offset = offset_delays[i];

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

float monopulse(Streams *streams, int *offset_delays, float *fractional_delays, int n_sensors) {
    float out[4][N_SAMPLES] = {0.0};

    unsigned n = 0;

    for (unsigned s = 0; s < n_sensors; s++) {
        if (VALID_SENSOR(s)) {
            
        
        
        int i = s % ELEMENTS;

        //float fraction = fractional_delays[i];
        //int offset = offset_delays[i];

        //std::cout << "frac: " << fraction << " offset: " << offset << std::endl;

        //float *signal = streams->get_signal(s, offset);
        float *signal = streams->get_signal(s, offset_delays[i]);

        int sector;
        if (in_sector(&first_sector[0], i)) {
            sector = 0;
        } else if (in_sector(&second_sector[0], i)) {
            sector = 1;
        } else if (in_sector(&third_sector[0], i)) {
            sector = 2;
        } else {
            sector = 3;
        }
        delay(&out[sector][0], signal, fractional_delays[i]);
        //delay(&out[sector][0], signal, fraction);
        }
    }

    const float norm = 1.0 / 16.0;
    float power[4] = {0.0, 0.0, 0.0, 0.0}; 
    for (int sector_index = 0; sector_index < 4; sector_index++) {
        for (int i = 0; i < N_SAMPLES; i++) {
            power[sector_index] += powf(out[sector_index][i] * norm, 2);
        }
    }

    double azimuth_error = (double)(power[0] - power[1]);
    double elevation_error = (double)(power[0] - power[3]);

    
    //double theta = PI_HALF - elevation_error;
    //double phi = azimuth_error;
    //if (theta > PI_HALF) {
    //    phi += PI;
    //    theta -= PI_HALF;
    //}

    

    double error = sqrt(powf(azimuth_error, 2) + powf(elevation_error, 2));

    //double azimuth = phi;
    //double elevation = PI_HALF - theta;
    double x = sin(azimuth_error);
    double y = sin(elevation_error);

    double xc = cos(azimuth_error);
    double yc = cos(elevation_error);
    //double phi = 
    double phi = atan2(y, x);
    ////double theta = cos(azimuth) 
    //double theta = acos(1.0 - )



    double theta = PI_HALF - asin(1.0 - pow(x, 2) - pow(y, 2));

    //std::cout << "Error: " << error << std::endl;

    return (power[0] + power[1] + power[2] + power[3]) / (float)N_SAMPLES + (error * 0.000001);

    //double phi, theta;


}


Particle::Particle(Antenna &antenna, Streams *streams, int n_sensors, int id) : antenna(antenna), streams(streams), n_sensors(n_sensors), id(id) {
    random();
    velocity_theta = drandom() - 0.5;
    velocity_phi = drandom() - 0.5;
    best_theta = theta;
    best_phi = phi;
    best_magnitude = compute(theta, phi, n_sensors);
}


void Particle::random() {
    phi = drandom() * (2.0 * PI);
    //theta = 0.0;
    theta = drandom() * PI_HALF;
    //phi = 0.0;
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

    //return monopulse(streams, &offset_delays[0], &fractional_delays[0], n_sensors);
    return beamform(streams, &offset_delays[0], &fractional_delays[0], n_sensors, id);

#endif
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
    return fmod(angle, TWO_PI);
    //return angle - TWO_PI * floor(angle / TWO_PI);
}

double smallestAngle(const double target, const double current) {
    return atan2(sin(target-current), cos(target-current));
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
        particles.emplace_back(this->antenna, pipeline->getStreams(), pipeline->get_n_sensors(), i%4);

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
        // If we convert to sphere with radius 0.5 we don't have to normalize it other than add
        // 0.5 to get the sphere in the first sector in the cartesian coordinate system
        Position position = spherical_to_cartesian(particle.best_theta, particle.best_phi, 1.0);
        //Position position = spherical_to_cartesian(particle.best_theta, particle.best_phi, 0.5);

        // Use the position values to plot over the heatmap
        int x = (int) (x_res * (position(X_INDEX) / 2.0 + 0.5));
        int y = (int) (y_res * (position(Y_INDEX) / 2.0 + 0.5));

#if 0
        if ((x < 0) || (x > x_res - 1) || (y < 0) || (y > y_res - 1)) {
            std::cout << "Invalid pos: (" << position(X_INDEX) << "," << position(Y_INDEX) << ") (" << x << ", " << y << ") " << std::endl;
        }
#endif
        

        heatmap->at<uchar>(x, y) = (255);
    }

    pso_lock.unlock();
}


void PSOWorker::loop() {
    std::cout << "Starting loop" << std::endl;
    while (looping && pipeline->isRunning()) {
        
        // Wait for incoming data
        pipeline->barrier();

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
                //particle.velocity_theta = current_velocity_weight * particle.velocity_theta + new_velocity_weight * (drandom() * smallestAngle(particle.best_theta, particle.theta) * LOCAL_AREA_RATIO + drandom() * smallestAngle(global_best_theta, particle.theta) * GLOBAL_AREA_RATIO);
                //particle.velocity_phi = current_velocity_weight * particle.velocity_phi + new_velocity_weight * (drandom() * (particle.best_phi - particle.phi) * LOCAL_AREA_RATIO + drandom() * (global_best_phi - particle.phi) * GLOBAL_AREA_RATIO);
                //particle.velocity_theta = current_velocity_weight * particle.velocity_theta + new_velocity_weight * (drandom() * (particle.best_theta - particle.theta) * LOCAL_AREA_RATIO + drandom() * (global_best_theta - particle.theta) * GLOBAL_AREA_RATIO);
                //particle.velocity_phi = current_velocity_weight * particle.velocity_phi + new_velocity_weight * (drandom() * smallestAngle(particle.best_phi, particle.phi) * LOCAL_AREA_RATIO + drandom() * smallestAngle(global_best_phi, particle.phi) * GLOBAL_AREA_RATIO);
                //particle.velocity_theta = current_velocity_weight * particle.velocity_theta + new_velocity_weight * (drandom() * (particle.best_theta - particle.theta) * LOCAL_AREA_RATIO + drandom() * (global_best_theta - particle.theta) * GLOBAL_AREA_RATIO);

                particle.velocity_phi = current_velocity_weight * particle.velocity_phi + new_velocity_weight * (smallestAngle(particle.best_phi, particle.phi) * LOCAL_AREA_RATIO + drandom() * smallestAngle(global_best_phi, particle.phi) * GLOBAL_AREA_RATIO);
                particle.velocity_theta = current_velocity_weight * particle.velocity_theta + new_velocity_weight * ((particle.best_theta - particle.theta) * LOCAL_AREA_RATIO + drandom() * (global_best_theta - particle.theta) * GLOBAL_AREA_RATIO);

                // Move based on velocity
                particle.theta += particle.velocity_theta * delta;
                particle.phi += particle.velocity_phi * delta; // * sin(particle.theta);
                
                
                // Limit the search area
                //particle.theta = wrapAngle(particle.theta);
                //particle.phi = clip(particle.phi, 0.0, PI_HALF);
                particle.phi = wrapAngle(particle.phi);
                particle.theta = clip(particle.theta, 0.0, PI_HALF);

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

        pso_lock.unlock();
    }

    std::cout << "Done loop" << std::endl;
}
