#ifndef WORKER_H
#define WORKER_H

#include <mutex>
#include <opencv2/opencv.hpp>// cv::Mat
#include <thread>

#include "pipeline.h"

#include "geometry.h"

/**
 * Random double between 0.0 and 1.0
 */
inline double drandom() {
    return static_cast<double>(rand()) / RAND_MAX;
}

/**
 * Target from the AWPU 
 */
struct Target {
    // Direction of target in spherical coordinate system
    Spherical direction;

    // Power level in the direction
    float power;

    // Pseudo-scientific value of how valid the target actually is
    float probability;

    bool operator==(const Target &other) const {
        return fabs(direction.phi - other.direction.phi) < 1e-2 && fabs(direction.theta - other.direction.theta) < 1e-2;
    }

};

/**
 * Worker types for beamforming algorithms
 */
enum worker_t {
    GENERIC,
    PSO,
    MIMO,
    SOUND,
    GRADIENT
};

class Worker {
public:
    // If the worker should be running
    bool *running;

    // If the worker is running
    bool looping;

    // Worker thread since parent thread is returned
    std::thread thread_loop;

    // Inherited pipeline
    Pipeline *pipeline;

    Worker(Pipeline *pipeline, Antenna &antenna, bool *running) : looping(true), pipeline(pipeline), antenna(antenna),
                                                                  running(running) {
        this->streams = pipeline->getStreams();
        thread_loop = std::thread(&Worker::loop, this);
    };

    ~Worker() {
        looping = false;
        //std::cout << "Waiting for thread to return" << std::endl;
        thread_loop.join();
        //std::cout << "thread returned" << std::endl;

        //std::cout << "Destroyed worker" << std::endl;
    };

    /**
     * Worker type
     */
    virtual worker_t get_type() {
        return worker_t::GENERIC;
    };

    /**
     * Getter for worker direction
     */
    Spherical getDirection() const {
        return direction;
    };

    /**
     * Getter for current targets
     */
    [[nodiscard]] std::vector<Target> getTargets() const {
        std::vector<Target> r_targets;
        for (int i = 0; i < tracking.size(); ++i) {
            r_targets.insert(r_targets.end(), tracking[i]);
        }
        return r_targets;
    };

    /**
     * Draw values onto a heatmap
     */
    void draw(cv::Mat *heatmap) {
        lock.lock();
        //std::cout << "Wrong drawer" << std::endl;
        populateHeatmap(heatmap);
        lock.unlock();
    };

protected:
    // Current direction
    Spherical direction;

    Streams *streams;
    Antenna &antenna;

    // Current tracking objects
    std::vector<Target> tracking;

    bool canContinue() {
        //std::cout << start <<" "<< pipeline->mostRecent() << std::endl;
        return (start == pipeline->mostRecent());
    }

    virtual void update() {
        std::cout << "Wrong update" << std::endl;
    };

    virtual void reset() {
    };

    virtual void populateHeatmap(cv::Mat *heatmap) {
    };

private:
    int start;

    // Lock to prevent multiple access from outside and inside
    std::mutex lock;

    /**
     * Worker main loop
     */
    void loop() {
        while (looping && pipeline->isRunning()) {
            // Wait for incoming data
            pipeline->barrier();

            start = pipeline->mostRecent();
            lock.lock();
            reset();
            update();
            lock.unlock();
        }
    }
};

#endif
