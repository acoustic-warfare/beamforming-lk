# cython: language_level=3
# distutils: language=c

import numpy as np
cimport numpy as np

# It's necessary to call "import_array" if you use any part of the numpy PyArray_* API.
np.import_array()

cimport delay
include "config.pxd"

def test():

    return test_delay()


cdef test_del():
    signals = np.arange(N_SAMPLES, dtype=np.float32)
    signals = np.linspace(0, 2 * np.pi, N_SAMPLES, dtype=np.float32)
    signals = np.sin(signals)
    cdef np.ndarray[np.float32_t, ndim=1, mode = 'c'] sig = np.ascontiguousarray(signals)
    
    out = np.zeros_like(signals)
    cdef np.ndarray[np.float32_t, ndim=1, mode = 'c'] res = np.ascontiguousarray(out)

    import matplotlib.pyplot as plt 

    plt.plot(signals[:10], label="original")
    for i in range(1, 10):
        h = i / 10
        out[:] = 0.0
        lerp_delay(&sig[0], &res[0], h)
        plt.plot(out[:10], label=f"delayed {h}")

    print(signals)
    print(out)
    print(res)

    


    plt.legend()
    plt.show()

def get_version():
    # test_del()
    return "0"