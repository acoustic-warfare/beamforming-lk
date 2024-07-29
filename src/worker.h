/** @file worker.h
 * @author Irreq
 * @brief TODO:
*/

#ifndef WORKER_H
#define WORKER_H

#include <mutex>
#include <opencv2/opencv.hpp>// cv::Mat
#include <thread>
#include <chrono>

#include "pipeline.h"

#include "geometry.h"

/**
 * @brief Random double between 0.0 and 1.0
 */
inline double drandom() {
    return static_cast<double>(rand()) / RAND_MAX;
}

/**
 * @class Target
 * @brief Target from the AWPU 
 */
struct Target {
    // Direction of target in spherical coordinate system
    Spherical direction;

    // Power level in the direction
    float power;

    // Pseudo-scientific value of how valid the target actually is
    float probability;

    // Time when target was first found
    std::chrono::time_point<std::chrono::high_resolution_clock> start;

    bool operator==(const Target &other) const {
        return fabs(direction.phi - other.direction.phi) < 1e-2 && fabs(direction.theta - other.direction.theta) < 1e-2;
    }

    //Target(Spherical direction, float power, float probability) {
    //    // Record the time of object creation
    //    start = std::chrono::high_resolution_clock::now();
    //};

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
    };

    ~Worker() {
        looping = false;
        //std::cout << "Waiting for thread to return" << std::endl;
        thread_loop.join();
        //std::cout << "thread returned" << std::endl;

        //std::cout << "Destroyed worker" << std::endl;
    };

    /**
     * @brief Worker type
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
     * @brief Getter for current targets
     */
    [[nodiscard]] std::vector<Target> getTargets() const {
        std::vector<Target> r_targets;
        for (int i = 0; i < tracking.size(); ++i) {
            r_targets.insert(r_targets.end(), tracking[i]);
        }
        return r_targets;
    };

    /**
     * @brief Draw values onto a heatmap
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
        std::cout << "Wrong reset" << std::endl;
    };

    virtual void populateHeatmap(cv::Mat *heatmap) {
        std::cout << "Wrong heatmap" << std::endl;
    };

    virtual void setup() {
        std::cout << "Wrong setup" << std::endl;
    };

    /**
     * @brief Worker main loop
     */
    void loop() {
        setup();
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

private:
    int start;

    // Lock to prevent multiple access from outside and inside
    std::mutex lock;

    
};

#endif
