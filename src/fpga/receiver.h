/** @file receiver.h
 * @author Irreq, Tuva
 * @brief Provides functions for establishing a UDP connection to an FPGA and receiving data.
 */

#ifndef BEAMFORMER_RECEIVER_H
#define BEAMFORMER_RECEIVER_H

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <iostream>

#define MAX_N_SENSORS 256
#define N_SAMPLES 256

#define RECEIVER_DEBUG 0

/**
 * @brief Struct representing a message received from the FPGA.
 */
typedef struct __attribute__((__packed__)) _msg {
    uint16_t frequency;
    uint8_t n_arrays;
    uint8_t version;
    uint32_t counter;
    int32_t stream[MAX_N_SENSORS];
} message;

/**
 * @brief Initializes a UDP connection to the FPGA.
 * @param address The IP address of the FPGA.
 * @param port The port number to connect to.
 * @return The socket descriptor on success, -1 on failure.
 */
int init_receiver(const char *address, const int port);

/**
 * @brief Receive a single sample and populate the message with the data
 * @param socket_desc The socket descriptor.
 * @param msg Pointer to the message structure to store the received message.
 * @return 0 on success, -1 on failure.
 */
int receive_message(int socket_desc, message *msg);

#endif // BEAMFORMER_RECEIVER_H