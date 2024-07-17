#ifndef WORKER_H
#define WORKER_H

#include <mutex>
#include <opencv2/opencv.hpp> // cv::Mat
#include "geometry.h"

inline double drandom() {
    return static_cast<double>(rand()) / RAND_MAX;
}

struct Target {
    Spherical direction;
    float power;
    float probability;
};

enum worker_t {

    PSO,
    MIMO,
    SOUND,
    GRADIENT
};

class Worker {
public:
    bool *running;
    bool looping;
    std::thread thread_loop;
    Pipeline *pipeline;
    Worker(Pipeline *pipeline, bool *running) : looping(true), pipeline(pipeline), running(running) {};

    ~Worker(){
        looping = false;
        std::cout << "Waiting for thread to return" << std::endl;
        thread_loop.join();
        std::cout << "thread returned" << std::endl;

        std::cout << "Destroyed worker" << std::endl;
    };

    worker_t get_type();

    Spherical getDirection() {
        return direction;
    };

    virtual void draw_heatmap(cv::Mat *heatmap)
     {
        std::cout << "Wrong drawer" << std::endl;
    };

    void loop() {};

    std::vector<Target> getTargets() { return tracking;};

protected:
    Spherical direction;
    std::vector<Target> tracking;
    std::mutex lock;
};

#endif