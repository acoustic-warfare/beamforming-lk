/** @file pipeline.h
 * @author Irreq, Tuva
 * @brief Establish a pipeline to the FPGA from the PC
*/

#ifndef PIPELINE_H
#define PIPELINE_H

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

#include "antenna.h"
#include "geometry.h"
#include "options.h"
#include "receiver.h"
#include "streams.hpp"

#define DEBUG_PIPELINE 1

/**
 * @class Pipeline
 * @brief Establish a pipeline to the FPGA from the PC
 */
class Pipeline {
public:
    Pipeline(const char *address, const int port, bool verbose = false);
    Pipeline(float frequency, Spherical direction, bool verbose = false);
    ~Pipeline();

    /**
     * @brief Connect beamformer to antenna
     * @return 0 if successful, -1 if error occured
     */
    int connect();

    /**
     * @brief Disconnect beamformer from antenna
     * @return 0 if successful, -1 if error occured
     */
    int disconnect();

    /**
     * @brief Checks if the pipeline is online
     * @return 0 if successful, -1 if error occured
     */
    int isRunning();

    /**
     * @brief check if no new data has been added
     * @return 0 if successful, -1 if error occured
     */
    int mostRecent();


    /**
     * @brief Wait until release
     */
    void barrier();

    /**
     * @brief Retrieves the steam of the pipeline
     * @return The stream attribute
     */
    Streams *getStreams() const;

    /**
     * @brief Fetches the number of sensors attribute
     * @return The number of sensors
     */
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

    bool verbose;

    bool synthetic;

    /**
     * @brief Allow the worker threads to continue
     */
    void release_barrier();

    /**
     * @brief Receive and fill ring buffers of new data
     */
    void receive_exposure();

    /**
     * @brief The main distributer of data to the threads (this is also a thread)
     */
    void producer();

    int connect_real();
    int connect_synthetic();

    void synthetic_producer();
};

#endif