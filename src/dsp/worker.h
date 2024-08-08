/** @file worker.h
 * @author Irreq
 * @brief Defines the Worker class and associated components for beamforming tasks.
 * Workers are what algorithm/audio processing method are running on the collected audio data.
 */

#ifndef WORKER_H
#define WORKER_H

#include <chrono>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>

#include "geometry.h"
#include "pipeline.h"

/**
 * @brief Random double between 0.0 and 1.0.
 * @return A random double value.
 */
inline double drandom() {
    return static_cast<double>(rand()) / RAND_MAX;
}

/**
 * @brief Detected targets from the AWPU.
 * Used for triagulation and track management. Updates tracks based on 
 * the targets’ positions, checking if they match with existing tracks 
 * or need to be added as new ones.
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
     * @brief Compares two Target objects for equality based on their direction.
     * @param other The target to compare with.
     * @return True if the targets have the same direction, false otherwise.
     */
    bool operator==(const Target &other) const {
        return fabs(direction.phi - other.direction.phi) < 1e-2 && fabs(direction.theta - other.direction.theta) < 1e-2;
    }

    /**
     * @brief Compares two Target objects for equality based on their direction.
     * @param other The target to compare with.
     * @return True if the targets have the same direction, false otherwise.
     */
    Target(Spherical direction, float power, float probability, std::chrono::time_point<std::chrono::high_resolution_clock> start) : 
    direction(direction), power(power), probability(probability), start(start){};
};

/**
 * Worker types for beamforming algorithms
 */
enum worker_t {
    GENERIC,///< Generic worker type.
    PSO,    ///< Particle Swarm Optimization worker type.
    MIMO,   ///< Multiple Input Multiple Output worker type.
    MISO,   ///< Multiple Input Single Output worker type.
    SOUND,  ///< Sound-based worker type.
    GRADIENT///< Gradient-based worker type.
};

/**
 * @class Worker
 * @brief Base class for beamforming workers.
 * Handles various beamforming algorithms, managing their execution in 
 * separate threads and processing targets.
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
     * @brief Constructs a Worker object.
     * @param pipeline Pointer to the pipeline object.
     * @param antenna Reference to the antenna object.
     * @param running Pointer to the running flag.
     */
    Worker(Pipeline *pipeline, Antenna &antenna, bool *running) : looping(true),
                                                                  pipeline(pipeline),
                                                                  antenna(antenna),
                                                                  running(running) {
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
     * @brief Get the worker type.
     * @return The worker type.
     */
    virtual worker_t get_type() {
        return worker_t::GENERIC;
    };

    /**
     * @brief Gets the direction of the worker.
     * @return The current direction in spherical coordinates.
     */
    Spherical getDirection() const {
        return direction;
    };

    /**
     * @brief Gets the current targets being tracked by the worker.
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
     * @brief Draws the current state onto a heatmap.
     * @param heatmap Pointer to the heatmap matrix.
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
     * @brief Checks if the worker can continue processing audio data.
     * @return True if the worker can continue, false otherwise.
     */
    bool canContinue() {
        return (start == pipeline->mostRecent());
    }

    /**
     * @brief TODO: Updates the worker's state.
     */
    virtual void update() {
        std::cout << "Wrong update" << std::endl;
    };

    /**
     * @brief TODO: Resets the workers state
     */
    virtual void reset() {
        std::cout << "Wrong reset" << std::endl;
    };

    /**
     * @brief TODO: Populates the heatmap with current data.
     * @param heatmap Pointer to the heatmap matrix.
     */
    virtual void populateHeatmap(cv::Mat *heatmap) {
        std::cout << "Wrong heatmap" << std::endl;
    };

    /**
     * @brief TODO: Sets up the worker before starting.
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
    /// Start index for processing 
    /// Self check against pipeline to only operate on newest data
    int start;

    /// Lock to prevent multiple access from outside and inside
    std::mutex lock;
};

#endif//WORKER_H