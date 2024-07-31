/** @file worker.h
 * @author Irreq
 * @brief TODO:
*/

#ifndef BEAMFORMER_WORKER_H
#define BEAMFORMER_WORKER_H

#include <chrono>
#include <mutex>
#include <opencv2/opencv.hpp>// cv::Mat
#include <thread>

#include "geometry.h"
#include "pipeline.h"

/**
 * @brief Random double between 0.0 and 1.0
 */
inline double drandom() {
    return static_cast<double>(rand()) / RAND_MAX;
}

/**
 * @brief Target from the AWPU 
 */
struct Target {
    /// Direction of target in spherical coordinate system
    Spherical direction;

    /// Power level in the direction
    float power;

    /// Pseudo-scientific value of how valid the target actually is
    float probability;

    /// Time when target was first found
    std::chrono::time_point<std::chrono::high_resolution_clock> start;

    /**
     * @brief 
     * @param other 
     * @return 
     */
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

/**
 * @class Worker
 * @brief TODO:
 */
class Worker {
public:
    /// If the worker should be running
    bool *running;

    /// If the worker is running
    bool looping;

    /// Worker thread since parent thread is returned
    std::thread thread_loop;

    /// Inherited pipeline
    Pipeline *pipeline;

    /**
     * @brief Construct a new Worker object
     * @param pipeline 
     * @param antenna 
     * @param running 
     */
    Worker(Pipeline *pipeline, Antenna &antenna, bool *running) : looping(true), pipeline(pipeline), antenna(antenna), running(running) {
        this->streams = pipeline->getStreams();
    };

    /**
     * @brief Destructor, cleanups of resources.
     */
    ~Worker() {
        looping = false;
        //std::cout << "Waiting for thread to return" << std::endl;
        thread_loop.join();
        //std::cout << "thread returned" << std::endl;

        //std::cout << "Destroyed worker" << std::endl;
    };

    /**
     * @brief Get the worker type
     * @return  
     */
    virtual worker_t get_type() {
        return worker_t::GENERIC;
    };

    /**
     * @brief Getter for worker direction
     * @return
     */
    Spherical getDirection() const {
        return direction;
    };

    /**
     * @brief Getter for current targets
     * @return
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
    /// Current direction
    Spherical direction;

    Streams *streams;
    Antenna &antenna;

    /// Current tracking objects
    std::vector<Target> tracking;

    /**
     * @brief 
     * @return 
     */
    bool canContinue() {
        //std::cout << start <<" "<< pipeline->mostRecent() << std::endl;
        return (start == pipeline->mostRecent());
    }

    /**
     * @brief 
     */
    virtual void update() {
        std::cout << "Wrong update" << std::endl;
    };

    /**
     * @brief 
     */
    virtual void reset() {
        std::cout << "Wrong reset" << std::endl;
    };

    /**
     * @brief 
     */
    virtual void populateHeatmap(cv::Mat *heatmap) {
        std::cout << "Wrong heatmap" << std::endl;
    };

    /**
     * @brief 
     */
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
    ///
    int start;

    /// Lock to prevent multiple access from outside and inside
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

#endif //BEAMFORMER_WORKER_H