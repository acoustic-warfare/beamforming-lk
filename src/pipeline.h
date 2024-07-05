#ifndef PIPELINE_H
#define PIPELINE_H

//#include "ring_buffer.h"
#include "options.h"
#include "receiver.h"
#include "streams.hpp"

//#include "ring_buffer.h"
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

using namespace std;

/**
 * @class Pipeline
 * @brief Establish a pipeline to the FPGA from the PC
 *
 */
class Pipeline {

public:
    std::atomic_int canPlot = 0;
    cv::Mat *magnitudeHeatmap;
    Pipeline(const char *ip_address_fpga);
    ~Pipeline();
    /**
   * Connect beamformer to antenna
   */
    int connect();

    /**
   * Disconnect beamformer from antenna
   */
    int disconnect();

    /**
   * Checks if the pipeline is online
   */
    int isRunning();

    /**
   * check if no new data has been added
   */
    int mostRecent();

    /**
   * Wait until release
   */
    void barrier();

    //Streams *getStreams(int stream_id);
    Streams *getStreams();

    int get_n_sensors() { return this->n_sensors; };

    int save_pipeline(std::string path);

private:
    int n_sensors;
    Streams *streams;
    const char *ip_address_fpga;
    int connected = 0;
    mutex pool_mutex;
    mutex barrier_mutex;
    condition_variable barrier_condition;
    int barrier_count = 0;

    thread connection;

    int modified = 0;

    int p = 0;

    /**
   * Allow the worker threads to continue
   */
    void release_barrier();

    /**
   * The main distributer of data to the threads (this is also a thread)
   */
    void producer(int n_sensors);//, int fpga_id);
};

#endif