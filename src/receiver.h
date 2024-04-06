#ifndef RECEIVER_H
#define RECEIVER_H

#include "config.h"
#include "ring_buffer.h"

#include <arpa/inet.h>
#include <cstdint>
#include <sys/socket.h>
#include <unistd.h>
#define HEADER_SIZE 2

// @brief FPGA Protocol Version 2
typedef struct _msg {
  // int32_t foo;
  // int32_t bar;
  int8_t version;
  int8_t n_arrays;
  int16_t frequency;
  int32_t counter;
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
int receive_offset(ring_buffer *rb);

/**
 * Put the latest frame into the ring_buffer
 */
// int receive(ring_buffer *rb);

#endif
