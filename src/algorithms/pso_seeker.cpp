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


float adaptive(Streams *streams, Antenna &antenna, int *offset_delays, float *fractional_delays) {
    float out[N_SAMPLES] = {0.0};

    //std::cout << "Usable: " << antenna.usable << std::endl;

    for (unsigned s = 0; s < antenna.usable; s++) {
        
        int i = antenna.index[s]; 
        float fraction = fractional_delays[i];

        //std::cout << "Using index: " << i << std::endl;

        float *signal = streams->get_signal(i, offset_delays[i]);
        delay(&out[0], signal, fraction);
        //delay_corrected(&out[0], signal, fraction, antenna.power_correction_mask[s]);
    }

    float power = 0.0f;
    float norm = 1 / (float) antenna.usable;

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


}


Particle::Particle(Antenna &antenna, Streams *streams, int id) : antenna(antenna), streams(streams), id(id) {
    random();
    this->velocity.theta = drandom() - 0.5;
    this->velocity.phi = drandom() - 0.5;
    this->best_magnitude = compute();
}


void Particle::random() {
    direction_current.phi = drandom() * (2.0 * PI);
    direction_current.theta = drandom() * PI_HALF;
}
float Particle::compute() {
    Eigen::VectorXf tmp_delays = steering_vector_spherical(antenna, direction_current.theta, direction_current.phi);

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

    return adaptive(streams, antenna, &offset_delays[0], &fractional_delays[0]);
}

void Particle::update() {
    float magnitude = compute();
    if (magnitude > best_magnitude) {
        best_magnitude = magnitude;
        direction_best = direction_current;
    }
}


PSOWorker::PSOWorker(Pipeline *pipeline, Antenna &antenna, bool *running, std::size_t swarm_size, std::size_t iterations) : Worker(pipeline, running), swarm_size(swarm_size), iterations(iterations) {
    this->global_best_magnitude = 0.0f;
    // Create antenna at origo
    this->antenna = antenna;
    thread_loop = std::thread(&PSOWorker::loop, this);
}

worker_t Worker::get_type() {
    return worker_t::PSO;
}

void PSOWorker::initialize_particles() {
    particles.clear();
    for (int i = 0; i < swarm_size; i++) {
        particles.emplace_back(this->antenna, pipeline->getStreams(), i%4);

        Particle &particle = particles.back();

        if (particle.best_magnitude > global_best_magnitude) {
            global_best_magnitude = particle.best_magnitude;
            direction = particle.direction_best;
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
        Cartesian position = Cartesian::convert(particle.direction_best, 1.0);

        // Use the position values to plot over the heatmap
        int x = (int) (x_res * (position.x / 2.0 + 0.5));
        int y = (int) (y_res * (position.y / 2.0 + 0.5));


        heatmap->at<uchar>(x, y) = 255;
    }

    Cartesian position = Cartesian::convert(direction, 1.0);
    //Position position = spherical_to_cartesian(global_best_theta, global_best_phi, 1.0);
    int x = (int) (x_res * (position.x / 2.0 + 0.5));
    int y = (int) (y_res * (position.y / 2.0 + 0.5));
    cv::circle(*heatmap, cv::Point(y,x), 3, cv::Scalar(255, 255, 255), cv::FILLED, 8, 0);

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
                particle.velocity.phi = current_velocity_weight * particle.velocity.phi + new_velocity_weight * (smallestAngle(particle.direction_best.phi, particle.direction_current.phi) * LOCAL_AREA_RATIO + drandom() * smallestAngle(direction.phi, particle.direction_current.phi) * GLOBAL_AREA_RATIO);
                particle.velocity.theta = current_velocity_weight * particle.velocity.theta + new_velocity_weight * ((particle.direction_best.theta - particle.direction_current.theta) * LOCAL_AREA_RATIO + drandom() * (direction.theta - particle.direction_current.theta) * GLOBAL_AREA_RATIO);

                // Move based on velocity
                particle.direction_current.theta += particle.velocity.theta * delta;
                particle.direction_current.phi += particle.velocity.phi * delta;// * sin(particle.theta);

                // Limit the search area
                particle.direction_current.phi = wrapAngle(particle.direction_current.phi);
                particle.direction_current.theta = clip(particle.direction_current.theta, 0.0, PI_HALF);

                // Search using new positions
                particle.update();

                // Update global best
                if (particle.best_magnitude > global_best_magnitude) {
                    global_best_magnitude = particle.best_magnitude;
                    direction = particle.direction_best;
                }
            }
        }

        pso_lock.unlock();
    }

    std::cout << "Done loop" << std::endl;
}
