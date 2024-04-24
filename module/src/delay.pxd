# cython: language_level=3
# distutils: language=c

include "pipeline.pxd"

cdef extern from "delay.h":
    int test_delay()
    # void delay()
    # int delay(ring_buffer *rb, float *out, float delay, int sensor_id)
    int naive_delay(ring_buffer *rb, float *out, float delay, int sensor_id)
    void lerp_delay(float *signal, float *out, float delay)

# cdef extern from "delay.h":
#     # int test_delay()
#     # void delay()
#     int naive_delay(ring_buffer *rb, float *out, float delay, int sensor_id)
#     void lerp_delay(float *signal, float *out, float delay)