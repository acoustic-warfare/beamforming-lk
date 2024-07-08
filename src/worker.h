#ifndef WORKER_H
#define WORKER_H

#include <opencv2/opencv.hpp> // cv::Mat

struct Direction {
    double azimuth;
    double elevation;
};

enum worker_t {

    PSO,
    MIMO,
    SOUND
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

    Direction getDirection() {
        return Direction(0, 0);
    };

    virtual void draw_heatmap(cv::Mat *heatmap)
     {
        std::cout << "Wrong drawer" << std::endl;
    };

    void loop() {};

protected:
    Direction direction;
};

#endif