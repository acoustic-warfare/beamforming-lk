#include "awpu.h"

AWProcessingUnit::AWProcessingUnit(const char *address, const int port, int verbose = 1) : verbose(verbose) {
    // Allocate memory for pipeline
    this->pipeline = new Pipeline(address, port);

    // Connect to FPGA
    this->pipeline->connect();
    this->running = false;
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

bool AWProcessingUnit::stop(const worker_t worker) {
    for (auto it = workers.begin(); it != workers.end();) {
        if ((*it)->get_type() == worker) {
            it = workers.erase(it);
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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

int main() {

    std::cout << "Connecting to FPGA" << std::endl;
#if 1
    AWProcessingUnit awpu = AWProcessingUnit("127.0.0.1", 21844);
#else 
    AWProcessingUnit awpu = AWProcessingUnit("10.0.0.1", 21875);
#endif

    std::cout << "Connected to FPGA" << std::endl;

    std::cout << "Starting PSO" << std::endl;
    awpu.start(PSO);

    std::cout << "Starting listening" << std::endl;
    //awpu.resume();

    // Create a window to display the beamforming data
    cv::namedWindow(APPLICATION_NAME, cv::WINDOW_NORMAL);
    cv::resizeWindow(APPLICATION_NAME, APPLICATION_WIDTH, APPLICATION_HEIGHT);

    cv::Mat frame(Y_RES, X_RES, CV_8UC1);
    cv::Mat colorFrame(Y_RES, X_RES, CV_8UC1);
    while (1) {
        awpu.draw_heatmap(&frame);
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

    awpu.pause();

    std::cout << "Stopping PSO" << std::endl;

    awpu.stop(PSO);

    std::cout << "Stopping program" << std::endl;

    return 0;
}