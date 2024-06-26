#include "ring_buffer.h"

/**
 * Lets a function from the outside sync the current index of the ring_buffer
 * such that it does not modify the current index each time data is being
 * written to it
 */
void offset_ring_buffer(ring_buffer *rb) {
  rb->index = (rb->index + N_SAMPLES) & (BUFFER_LENGTH - 1);
}

void set_ring_buffer_imm(ring_buffer *rb, size_t m) {
  rb->index = m;
}

//inline void write_buffer(ring_buffer *rb, float *frame) 

inline void write_buffer(ring_buffer *rb, float *frame) {

  int inverted = 0;

  // Array is daisy-chained, flip even columns
  for (unsigned i = 0; i < N_SENSORS; i++) {
    if (i % COLUMNS == 0) {
      inverted = !inverted;
    }

    if (inverted) {
      rb->data[i][rb->index] = frame[i];
    } else {
      rb->data[i][rb->index] = frame[COLUMNS * (1 + i / COLUMNS) - i % COLUMNS];
    }
  }

  rb->index = (rb->index + 1) & (BUFFER_LENGTH - 1);
}