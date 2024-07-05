#include "awpu.h"

AWProcessingUnit::AWProcessingUnit(const char *ip_address) : ip_address(ip_address) {
    this->pipeline = new Pipeline(ip_address);
    this->pipeline->connect();
    this->running = false;
}

AWProcessingUnit::~AWProcessingUnit() {
    std::cout << "Destructing AWPU" << std::endl;
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
    if (this->running) {
        this->running = false;
    }
}

void AWProcessingUnit::resume() {
    if (!this->running) {
        this->running = true;
    }
}

void AWProcessingUnit::draw_heatmap(cv::Mat *heatmap) {
    workers[0]->draw_heatmap(heatmap);
}

#include <unistd.h>


int main() {

    std::cout << "Connecting to FPGA" << std::endl;
    AWProcessingUnit awpu = AWProcessingUnit("127.0.0.1");

    std::cout << "Connected to FPGA" << std::endl;

    std::cout << "Starting PSO" << std::endl;
    awpu.start(PSO);

    std::cout << "Starting listening" << std::endl;
    awpu.resume();

    // Create a window to display the beamforming data
    cv::namedWindow(APPLICATION_NAME, cv::WINDOW_NORMAL);
    cv::resizeWindow(APPLICATION_NAME, APPLICATION_WIDTH, APPLICATION_HEIGHT);

    cv::Mat frame(Y_RES, X_RES, CV_8UC1);
    while (1) {
        awpu.draw_heatmap(&frame);
        // Apply color map
        // Blur the image with a Gaussian kernel
        cv::GaussianBlur(frame, frame,
                             cv::Size(BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE), 0);
        cv::applyColorMap(frame, frame, cv::COLORMAP_JET);
        cv::imshow(APPLICATION_NAME, frame);
        if (cv::waitKey(1) == 'q') {
            std::cout << "Stopping application..." << std::endl;
            break;
        }
    }

    //usleep(2e6);

    std::cout << "Pausing listening" << std::endl;

    awpu.pause();

    std::cout << "Stopping PSO" << std::endl;

    awpu.stop(PSO);

    std::cout << "Stopping program" << std::endl;

    return 0;
}