#include "awpu.h"

AWProcessingUnit::AWProcessingUnit(const char *address, const int port, int verbose = 1) : verbose(verbose) {
    // Allocate memory for pipeline
    this->pipeline = new Pipeline(address, port);

    // Connect to FPGA
    this->pipeline->connect();
    this->running = false;

    for (int a = 0; a < this->pipeline->get_n_sensors() / ELEMENTS; a++) {
        antennas.emplace_back();
    }
}

AWProcessingUnit::~AWProcessingUnit() {
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

            job = (Worker *) new PSOWorker(pipeline, &running, SWARM_SIZE, SWARM_ITERATIONS);
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
 * and calculates how to compensate for it. 
 */
void AWProcessingUnit::calibrate(const float reference_power_level = 1e-4) {
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

        // Find out the median
        float medians[ELEMENTS];

        memcpy(&medians[0], &power[0], sizeof(float) * ELEMENTS);

        std::sort(std::begin(medians), std::end(medians));

        float median = (medians[ELEMENTS / 2] + medians[ELEMENTS / 2 + 1]) / 2.0;

        
        float mmin = 1.0; // We have big problems is anything is above 1.0
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

#include <unistd.h>

#if 1
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

int main() {

    std::cout << "Connecting to FPGA" << std::endl;
#if 0
    AWProcessingUnit awpu = AWProcessingUnit("127.0.0.1", 21844);
#else 
    AWProcessingUnit awpu5 = AWProcessingUnit("10.0.0.1", 21875);
    AWProcessingUnit awpu8 = AWProcessingUnit("10.0.0.1", 21878);
#endif

    std::cout << "Connected to FPGA" << std::endl;

    std::cout << "Starting PSO" << std::endl;
    awpu5.start(PSO);
    awpu8.start(PSO);

    std::cout << "Starting listening" << std::endl;
    //awpu.resume();

    // Create a window to display the beamforming data
    cv::namedWindow(APPLICATION_NAME, cv::WINDOW_NORMAL);
    cv::resizeWindow(APPLICATION_NAME, APPLICATION_WIDTH, APPLICATION_HEIGHT);

    cv::Mat frame(Y_RES, X_RES, CV_8UC1);
    cv::Mat colorFrame(Y_RES, X_RES, CV_8UC1);
    while (1) {
        awpu5.draw_heatmap(&frame);
        //awpu8.draw_heatmap(&frame);
        // Apply color map
        // Blur the image with a Gaussian kernel

        cv::GaussianBlur(frame, colorFrame,
                             cv::Size(BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE), 0);
        cv::applyColorMap(colorFrame, colorFrame, cv::COLORMAP_JET);
        cv::imshow(APPLICATION_NAME, colorFrame);
        if (cv::waitKey(1) == 'q') {
            std::cout << "Stopping application..." << std::endl;
            break;
        }
    }

    std::cout << "Pausing listening" << std::endl;

    awpu5.pause();
    awpu8.pause();

    std::cout << "Stopping PSO" << std::endl;

    awpu5.stop(PSO);
    awpu8.stop(PSO);

    std::cout << "Stopping program" << std::endl;

    return 0;
}

#else 


int main() {

    std::cout << "Connecting to FPGA" << std::endl;
    AWProcessingUnit awpu = AWProcessingUnit("10.0.0.1", 21878);
    std::cout << "Connected to FPGA" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    awpu.calibrate();
    awpu.pause();
    return 0;
}

#endif