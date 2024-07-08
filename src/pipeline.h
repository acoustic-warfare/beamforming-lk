#ifndef PIPELINE_H
#define PIPELINE_H

#include "options.h"
#include "receiver.h"
#include "streams.hpp"

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#define DEBUG_PIPELINE 1

/**
 * @class Pipeline
 * @brief Establish a pipeline to the FPGA from the PC
 *
 */
class Pipeline {
public:
    Pipeline(const char *address, const int port);
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

    Streams *getStreams();

    int get_n_sensors() { return this->n_sensors; };

    int save_pipeline(std::string path);

private:
    // Buffer for storing incoming UDP mic data
    float **exposure_buffer;

    // UDP socket descriptor
    int socket_desc;

    // FPGA ip address
    const char *address;

    // FPGA port
    const int port;

    // Message for receiving UDP packet
    message msg;

    // Number of sensors in current constallation
    int n_sensors;

    // Ring buffer for storing data 
    Streams *streams;

    // Receiver thread
    std::thread receiver_thread;

    // If pipeline is connected to FPGA
    int connected = 0;

    // Mutex for workers
    std::mutex pool_mutex;

    // Barrier for workers
    std::mutex barrier_mutex;

    // Status of barrier
    std::condition_variable barrier_condition;

    // Number of current stopped workers
    int barrier_count = 0;

    // Check if new data was received
    int modified = 0;

    /**
     * Allow the worker threads to continue
     */
    void release_barrier();

    /**
     * Receive and fill ring buffers of new data
     */
    void receive_exposure();

    /**
     * The main distributer of data to the threads (this is also a thread)
     */
    void producer();
};

#endif