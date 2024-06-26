# cython: language_level=3
# distutils: language=c++

cimport pipeline

from config cimport *




import cython
cimport cython

include "config.pxd"


import numpy as np
cimport numpy as np
# It's necessary to call "import_array" if you use any part of the numpy PyArray_* API.
np.import_array()

cdef class CyPipeline:

    cdef Pipeline* _obj
    cdef ring_buffer rb
    cdef public np.ndarray frame


    def __init__(self):
        self._obj = new Pipeline()
        if self._obj == NULL:
            raise MemoryError("Not enough memory for CyPipeline")

    def __del__(self):
        del self._obj


# property rb 
#     def __get__(self):
#         return self._obj.getRingBuffer()
#     def __set__(self, int obj):
#         raise RuntimeError("Operation not permitted")


    # ---- Wrappers ----

    def connect(self) -> int:
        return self._obj.connect()

    def disconnect(self) -> int:
        return self._obj.disconnect()

    def getRingBuffer(self) -> ring_buffer:
        # cdef ring_buffer rb = self._obj.getRingBuffer()

        # self.frame.data = rb.#self._obj.getRingBuffer()

        return self.frame

    def isRunning(self) -> int:
        return self._obj.isRunning()


    def mostRecent(self) -> int:
        """
        Wrapper for most recent transmission
        """
        return self._obj.mostRecent()

    
    

# cdef class _PipeLine:
#     cdef ring_buffer *rb
#     cdef public bint running
#     cdef public np.ndarray frame
#
#     def __cinit__(self):
#         # Initialize the out_array in the constructor
#         self.rb = pipeline.create_ring_buffer()
#         self.running = True
#         self.frame = np.zeros((N_SENSORS, BUFFER_LENGTH), dtype=np.float32)
#         
#     @cython.boundscheck(False)
#     @cython.wraparound(False)
#     def get_data(self, np.ndarray[np.float32_t, ndim=2, mode="c"] out):
#         """Copy the ringbuffer into data"""
#         read_buffer_all(self.rb, <float (*)[BUFFER_LENGTH]> out.data)
#     
#     @cython.boundscheck(False)
#     @cython.wraparound(False)
#     def store(self, np.ndarray[np.float32_t, ndim=1, mode="c"] arr):
#         """Store a vector to the ringbuffer"""
#         write_buffer_single(self.rb, <float (*)> arr.data)
#     
#     @cython.boundscheck(False)
#     @cython.wraparound(False)
#     def store_all(self, np.ndarray[np.float32_t, ndim=2, mode="c"] arr) -> None:
#         """Store multiple vectors into a ringbuffer""" 
#         write_buffer_all(self.rb, <float (*)[N_SAMPLES]> arr.data)
#
#     @cython.boundscheck(False)
#     @cython.wraparound(False)
#     def latest(self) -> np.ndarray:
#         """Receive the latest samples into a buffer
#
#         """
#         read_buffer_all(self.rb, <float (*)[BUFFER_LENGTH]> self.frame.data)
#         return self.frame
#
#     @cython.boundscheck(False)
#     @cython.wraparound(False)
#     def get_last_frame(self):
#         """Retrieve the last frame as an array"""
#         return self.latest()[:, -N_SAMPLES:]
#
#     @cython.boundscheck(False)
#     @cython.wraparound(False)
#     def delay_last_frame(self, np.ndarray delay_vector) -> np.ndarray:
#         """Delay the latest samples for a specific amount"""
#         out = np.zeros((N_SENSORS, N_SAMPLES), dtype=np.float32)
#         cdef np.ndarray[np.float32_t, ndim=2, mode = 'c'] res = np.ascontiguousarray(out)
#
#         for i in range(N_SENSORS):
#             error = pipeline.naive_delay(self.rb, &res[i, 0], delay_vector[i], i)
#
#             if error:
#                 raise RuntimeError("Unable to delay, see stacktrace")
#
#         return out
#
# # Expose the cython class
# class Pipeline(_PipeLine):
#     def __init__(self):
#         super().__init__()
