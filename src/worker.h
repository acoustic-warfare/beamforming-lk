#ifndef WORKER_H
#define WORKER_H

#include <mutex>
#include <opencv2/opencv.hpp> // cv::Mat

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

protected:
    Spherical direction;
    std::mutex lock;
};

#endif