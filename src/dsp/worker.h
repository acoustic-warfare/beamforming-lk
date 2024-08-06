/** @file worker.h
 * @author Irreq
 * @brief TODO:
*/

#ifndef BEAMFORMER_WORKER_H
#define BEAMFORMER_WORKER_H

#include <chrono>
#include <mutex>
#include <opencv2/opencv.hpp>
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

    Target(Spherical direction, float power, float probability, std::chrono::time_point<std::chrono::high_resolution_clock> start) :
    direction(direction), power(power), probability(probability), start(start) {};
};

/**
 * Worker types for beamforming algorithms
 */
enum worker_t {
    GENERIC,
    PSO,
    MIMO,
    MISO,
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
        thread_loop.join();
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
     * @return Current tracked targets (if any)
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
        populateHeatmap(heatmap);
        lock.unlock();
    };

    /**
     * @brief Steer the worker to a specific direction
     */
    virtual void steer(Spherical direction) {};

protected:
    /// Current direction
    Spherical direction;

    /// Incoming data
    Streams *streams;

    /// @brief steerable antenna with valid sensors
    Antenna &antenna;

    /// Current tracking objects
    std::vector<Target> tracking;

    /**
     * @brief 
     * @return 
     */
    bool canContinue() {
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
    /// Self check against pipeline to only operate on newest data
    int start;

    /// Lock to prevent multiple access from outside and inside
    std::mutex lock;
};

#endif //BEAMFORMER_WORKER_H