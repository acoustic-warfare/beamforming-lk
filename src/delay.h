#ifndef DELAY_H
#define DELAY_H

#include "config.h"
#include "ring_buffer.h"

#include <cmath>

#define SAFETY_CHECK 0

int naive_delay(ring_buffer *rb, float *out, float delay, int sensor_id) {
  double _offset;
  float fraction;

  fraction = (float)modf((double)delay, &_offset);

  int offset = (int)_offset;

#if SAFETY_CHECK
  if ((offset >= BUFFER_LENGTH - N_SAMPLES) || (offset < 0)) {
    printf("Out of bounds delay, increase buffer size\n");
    exit(1);
  }
#endif

  int prev, current, start;

  float *signal = &rb->data[sensor_id][0];

  // Start index begins from the start of the latest N_SAMPLES - offset and
  // forwards
  start = rb->index + BUFFER_LENGTH - N_SAMPLES - offset;

  for (int i = 0; i < N_SAMPLES; i++) {
    current = start + i;
    prev = current - 1;

    current &= (BUFFER_LENGTH - 1);
    prev &= (BUFFER_LENGTH - 1);

    // printf("(%d %d) Prev: %f Current: %f \n", prev, current, signal[prev],
    // signal[current]);

    out[i] += signal[current] + fraction * (signal[prev] - signal[current]);
  }

  return 0;
}

#endif
