# cython: language_level=3
# distutils: language=c++

include "config.pxd"

cdef extern from "receiver.h":
    ctypedef struct ring_buffer:
        int index
        float data[N_SENSORS][BUFFER_LENGTH]

    void write_buffer(ring_buffer *rb, float *frame);
    void offset_ring_buffer(ring_buffer *rb);
