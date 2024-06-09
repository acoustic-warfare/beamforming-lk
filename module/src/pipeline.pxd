# cython: language_level=3
# distutils: language=c++

include "config.pxd"
include "ring_buffer.pxd"

cdef extern from "pipeline.h":
    cppclass Pipeline:
        Pipeline()
        int connect()
        int disconnect()
        int isRunning()
        int mostRecent()
        void barrier()

        ring_buffer &getRingBuffer()
        int save_pipeline(str path)
