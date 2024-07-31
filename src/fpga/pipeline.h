/** @file pipeline.h
 * @author Irreq, Tuva
 * @brief Establish a pipeline to the FPGA from the PC
*/

#ifndef BEAMFORMER_PIPELINE_H
#define BEAMFORMER_PIPELINE_H

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
#include "receiver.h"
#include "streams.hpp"

#define MAX_VALUE_FLOAT 8388608.0

/**
 * @class Pipeline
 * @brief Establishes and manages a communication pipeline between the PC and FPGA.
 */
class Pipeline {
public:
    /**
     * @brief Constructor to initialize the pipeline with FPGA address and port.
     * @param address The IP address of the FPGA.
     * @param port The port number of the FPGA.
     * @param verbose Flag to enable verbose output.
     */
    Pipeline(const char *address, const int port, bool verbose = false);

    /**
     * @brief Constructor to initialize the pipeline with frequency and direction.
     * @param frequency The frequency of the signal.
     * @param direction The direction of the signal in spherical coordinates.
     * @param verbose Flag to enable verbose output.
     */
    Pipeline(float frequency, Spherical direction, bool verbose = false);

    /**
     * @brief Destructor, cleanups of resources.
     */
    ~Pipeline();

    /**
     * @brief Connect beamformer to antenna.
     * @return 0 if successful, -1 if error occured.
     */
    int connect();

    /**
     * @brief Disconnect beamformer from antenna.
     * @return 0 if successful, -1 if error occured.
     */
    int disconnect();

    /**
     * @brief Checks if the pipeline is online.
     * @return 0 if successful, -1 if error occured.
     */
    int isRunning();

    /**
     * @brief Check if no new data has been added.
     * @return 0 if successful, -1 if error occured.
     */
    int mostRecent();


    /**
     * @brief Wait until release, waits until data exists to be read.
     */
    void barrier();

    /**
     * @brief Retrieves the steam of the pipeline.
     * @return The stream attribute.
     */
    Streams *getStreams() const;

    /**
     * @brief Fetches the number of sensors attribute.
     * @return The number of sensors.
     */
    int get_n_sensors() { return this->n_sensors; };

    /**
     * @brief Saves the current pipeline configuration to a file.
     * @param path The file path where the configuration should be saved.
     * @return 0 if successful, -1 if error occured.
     */
    int save_pipeline(std::string path);


private:
    /// Buffer for storing incoming UDP mic data
    float **exposure_buffer;

    /// UDP socket descriptor
    int socket_desc;

    /// FPGA ip address
    const char *address;

    /// FPGA port
    const int port;

    /// Message for receiving UDP packet
    message msg;

    /// Number of sensors in current constallation
    int n_sensors;

    /// Ring buffer for storing data
    Streams *streams;

    /// Receiver thread
    std::thread receiver_thread;

    /// If pipeline is connected to FPGA
    int connected = 0;

    /// Mutex for workers
    std::mutex pool_mutex;

    /// Barrier for workers
    std::mutex barrier_mutex;

    /// Status of barrier
    std::condition_variable barrier_condition;

    /// Number of current stopped workers
    int barrier_count = 0;

    /// Check if new data was received
    int modified = 0;

    /// Verbose for extra logging
    bool verbose;

    /// Flag indicating if synthetic data is used
    bool synthetic;

    /**
     * @brief Allow the worker threads to continue.
     */
    void release_barrier();

    /**
     * @brief Receive and fill ring buffers of new data.
     */
    void receive_exposure();

    /**
     * @brief The main distributer of data to the threads (this is also a thread).
     */
    void producer();

    /**
     * @brief Connects to the FPGA using a real connection.
     * @return 0 if successful, -1 if error occured.
     */
    int connect_real();

    /**
     * @brief Connects to the FPGA using synthetic data.
     * @return 0 if successful, -1 if error occured.
     */
    int connect_synthetic();

    /**
     * @brief Generates synthetic data for testing purposes.
     */
    void synthetic_producer();
};

#endif //BEAMFORMER_PIPELINE_H