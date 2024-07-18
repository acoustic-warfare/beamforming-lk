#ifndef WORKER_H
#define WORKER_H

#include <mutex>
#include <opencv2/opencv.hpp>// cv::Mat

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

    bool operator==(const Target &other) const{
        return direction.toCartesian().isApprox(other.direction.toCartesian());
    }

};

/**
 * Worker types for beamforming algorithms
 */
enum worker_t {
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
    Worker(Pipeline *pipeline, bool *running) : looping(true), pipeline(pipeline), running(running){};

    ~Worker() {
        looping = false;
        std::cout << "Waiting for thread to return" << std::endl;
        thread_loop.join();
        std::cout << "thread returned" << std::endl;

        std::cout << "Destroyed worker" << std::endl;
    };

    /**
     * Worker type
     */
    worker_t get_type();

    /**
     * Getter for worker direction
     */
    Spherical getDirection() const {
        return direction;
    };

    /**
     * Getter for current targets
     */
    std::vector<Target> getTargets() const { return tracking; };

    /**
     * Draw values onto a heatmap
     */
    virtual void draw_heatmap(cv::Mat *heatmap) {
        std::cout << "Wrong drawer" << std::endl;
    };

    /**
     * Worker main loop
     */
    virtual void loop() {};


protected:
    // Current direction
    Spherical direction;

    // Current tracking objects
    std::vector<Target> tracking;

    // Lock to prevent multiple access from outside and inside
    std::mutex lock;
};

#endif