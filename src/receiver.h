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

#define HEADER_SIZE 2

// @brief FPGA Protocol Version 2
typedef struct __attribute__((__packed__)) _msg {
    uint16_t frequency;
    uint8_t n_arrays;
    uint8_t version;
    uint32_t counter;
    int32_t stream[MAX_N_SENSORS];
} message;

/**
 * Start the UDP receiver
 */
int init_receiver(const char *address, const int port);

/**
 * Receive an entire frame and add it to ringbuffer with an offset
 */
int receive_message(int socket_desc, message *msg);

#endif