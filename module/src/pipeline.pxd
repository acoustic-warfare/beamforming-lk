# cython: language_level=3
# distutils: language=c

include "config.pxd"

cdef extern from "ring_buffer.h":
    ctypedef struct ring_buffer:
        int index
        float data

    ring_buffer *create_ring_buffer()

    ring_buffer *destroy_ring_buffer(ring_buffer *rb)

    void write_buffer(ring_buffer *rb, float *in_)

    void write_buffer_single(ring_buffer *rb, float *data)

    void read_mcpy(ring_buffer *rb, float *out)

    void write_buffer_all(ring_buffer *rb, float (*data)[N_SAMPLES])

    void read_buffer_all(ring_buffer *rb, float (*out)[BUFFER_LENGTH])
    

cdef extern from "delay.h":
    int naive_delay(ring_buffer *rb, float *out, float delay, int sensor_id)
    void lerp_delay(float *signal, float *out, float delay)