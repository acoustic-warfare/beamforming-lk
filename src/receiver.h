/** @file receiver.h
 * @author Irreq, Tuva
 * @brief TODO:
*/

#ifndef RECEIVER_H
#define RECEIVER_H

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <iostream>

#include "config.h"

#define RECEIVER_DEBUG 0

// @brief FPGA Protocol Version 2
typedef struct __attribute__((__packed__)) _msg {
    uint16_t frequency;
    uint8_t n_arrays;
    uint8_t version;
    uint32_t counter;
    int32_t stream[MAX_N_SENSORS];
} message;

/**
 * @brief Initializes a UDP connection to the FPGA.
 * 
 * @param address The IP address of the FPGA.
 * @param port The port number to connect to.
 * @return int The socket descriptor on success, -1 on failure.
 */
int init_receiver(const char *address, const int port);

/**
 * @brief Receive a single sample and populate the message with the data
 * @param socket_desc The socket descriptor.
 * @param msg Pointer to the message structure to store the received message.
 * @return int 0 on success, -1 on failure.
 */
int receive_message(int socket_desc, message *msg);

#endif // RECEIVER_H