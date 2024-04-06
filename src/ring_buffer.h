#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include "config.h"
#include <cstddef>
#include <cstdint>

struct ring_buffer {
  std::size_t index;
  float data[N_SENSORS][BUFFER_LENGTH];
};

void write_buffer(ring_buffer *rb, float *frame);
void offset_ring_buffer(ring_buffer *rb);
#endif // !RING_BUFFER_H
