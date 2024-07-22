#include "awpu.h"

#include "audio/audio_wrapper.h"

AWProcessingUnit::AWProcessingUnit(const char *address, const int port, int verbose) : verbose(verbose) {
    // Allocate memory for pipeline
    this->pipeline = new Pipeline(address, port);

    // Connect to FPGA
    this->pipeline->connect();
    this->running = false;

    for (int a = 0; a < this->pipeline->get_n_sensors() / ELEMENTS; a++) {
        Antenna antenna = create_antenna(Position(0, 0, 0), COLUMNS, ROWS, DISTANCE);
        //antennas.emplace_back();
        antennas.push_back(antenna);
    }
}

AWProcessingUnit::~AWProcessingUnit() {

    pause();

    stop_audio();

    for (auto it = workers.begin(); it != workers.end();) {
        it = workers.erase(it);
        delete (*it);
    }

    if (verbose) {
        std::cout << "Destructing AWPU" << std::endl;
    }

    pipeline->disconnect();
    delete pipeline;

    for (auto &job: workers) {
        delete job;
    }
}

bool AWProcessingUnit::start(const worker_t worker) {
    Worker *job;
    switch (worker) {
        case PSO:
            job = (Worker *) new PSOWorker(pipeline, antennas[0], &running, SWARM_SIZE, SWARM_ITERATIONS);
            break;
        case MIMO:
            job = nullptr;
            break;
        case SOUND:
            job = nullptr;
            break;

        default:
            return false;
    }

    if (job) {
        workers.push_back(job);
        return true;
    } else {
        return false;
    }
}

/**
 * Autocalibrate antennas by referencing each element and check if its too big or small 
 * and calculates how to compensate for it. Remember that the correction is only valid for what is in the buffers
 * this is not an absolute measure and should be used accordingly
 */
void AWProcessingUnit::calibrate(const float reference_power_level) {
    Streams *streams = pipeline->getStreams();


    float signals[4][ELEMENTS][N_ITEMS_BUFFER];

    int n_sensors = pipeline->get_n_sensors();

    // Get a snapshot of the pipeline
    for (int a = 0; a < antennas.size(); a++) {

        for (int s = 0; s < ELEMENTS; s++) {
            streams->read_stream(s + a * ELEMENTS, &signals[a][s][0]);
        }
    }

    // Calibrate each connected antenna individually
    for (int a = 0; a < antennas.size(); a++) {
        Antenna &antenna = antennas[a];
        float power[ELEMENTS];

        float mean = 0.0;

        // Compute all power levels
        for (int s = 0; s < ELEMENTS; s++) {
            float power_value = 0.0;
            for (int i = 0; i < N_ITEMS_BUFFER; i++) {
                power_value += powf(signals[a][s][i], 2);
            }

            power_value /= (float) N_ITEMS_BUFFER;

            mean += power_value;
            power[s] = power_value;
        }

        // Find out the median we assume the median is ok
        float medians[ELEMENTS];

        memcpy(&medians[0], &power[0], sizeof(float) * ELEMENTS);

        std::sort(std::begin(medians), std::end(medians));

        float median = (medians[ELEMENTS / 2] + medians[ELEMENTS / 2 + 1]) / 2.0;


        float mmin = 1.0;// We have big problems is anything is above 1.0
        float mmax = 0.0;

        int index[ELEMENTS];
        int count = 0;

        // Filter elements that are too off
        for (int s = 0; s < ELEMENTS; s++) {
            float current_power_level = power[s];
            float diff = fabs(power[s] - median);
            if (diff > 1e-4) {// Too small
                //std::cout << " [BAD Large]";
            } else if (power[s] < median * 1e-3) {// Too big
                //std::cout << " [BAD Small]";
            } else {
                if (current_power_level > mmax) {
                    mmax = current_power_level;
                }

                if (current_power_level < mmin) {
                    mmin = current_power_level;
                }

                index[count] = s;
                count++;
                mean += current_power_level;
            }
        }

        mean /= (float) count;

        antenna.usable = count;
        antenna.median = median;
        antenna.mean = mean;

        // Allocate memory for indexes and power level correction
        if (count > 0) {
            antenna.power_correction_mask = new float[count];
            antenna.index = new int[count];
        }

        for (int s = 0; s < count; s++) {
            int valid_index = index[s];
            float current_valid_power = power[valid_index];

            antenna.index[s] = valid_index;

            antenna.power_correction_mask[s] = reference_power_level / current_valid_power;
        }

        std::cout << "Calibrated antenna " << a << " Usable: " << antenna.usable << " Mean: " << antenna.mean << " Median: " << antenna.median << std::endl;

        for (int s = 0; s < antenna.usable; s++) {
            std::cout << "Mic: " << antenna.index[s] << " Correction: " << antenna.power_correction_mask[s] << std::endl;
        }

        std::cout << std::endl;
    }
}

bool AWProcessingUnit::stop(const worker_t worker) {
    for (auto it = workers.begin(); it != workers.end();) {
        if ((*it)->get_type() == worker) {
            //std::cout << "Stopping worker from AWPU Workers" << std::endl;
            //(*it)->looping = false;
            it = workers.erase(it);
            delete (*it);
            return true;
        }

        else {
            ++it;
        }
    }

    return false;
}

void AWProcessingUnit::pause() {
    this->running = false;
}

void AWProcessingUnit::resume() {
    this->running = true;
}

void AWProcessingUnit::draw_heatmap(cv::Mat *heatmap) {
    workers[0]->draw_heatmap(heatmap);
}

void AWProcessingUnit::play_audio() {
    audioWrapper = new AudioWrapper(*pipeline->getStreams());
    audioWrapper->start_audio_playback();
}

void AWProcessingUnit::stop_audio() {
    if (audioWrapper) {
        std::cout << "Stopping audio" << std::endl;
        audioWrapper->stop_audio_playback();
        std::cout << "Deleting audiowrapper" << std::endl;
        delete audioWrapper;
    }
}

Spherical AWProcessingUnit::target() {
    return workers[0]->getDirection();
}