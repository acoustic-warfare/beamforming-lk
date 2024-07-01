#ifndef RECEIVER_H
#define RECEIVER_H

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <iostream>

#include "config.h"
#include "streams.hpp"
#define HEADER_SIZE 2

// @brief FPGA Protocol Version 2
typedef struct __attribute__((__packed__)) _msg {
    uint16_t frequency;
    uint8_t n_arrays;
    uint8_t version;
    uint32_t counter;
    int32_t stream[N_SENSORS];
} message;

/**
 * To be called on stopping
 */
void stop_receiving();

/**
 * Start the UDP receiver
 */
int init_receiver();

/**
 * Receive an entire frame and add it to ringbuffer with an offset
 */
int receive_offset(Streams *streams);

/**
 * Put the latest frame into the ring_buffer
 */
int number_of_sensors();

int receive_exposure(Streams *streams);

#endif
