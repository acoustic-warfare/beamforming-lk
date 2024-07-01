#ifndef DELAY_H
#define DELAY_H

#include <algorithm>
#include <cmath>

#include "config.h"

#define SAFETY_CHECK 0


int virtual_delay(float *signal_, float *out, float delay) {
    double _offset;
    float fraction;

    fraction = (float) modf((double) delay, &_offset);

    int offset = N_SAMPLES - (int) _offset;

    float *signal = (float *) ((char *) signal_ + offset * sizeof(float));

    //out[0] = 1.0;

    for (int i = 0; i < N_SAMPLES; i++) {
        out[i] += signal[i + 1] + fraction * (signal[i] - signal[i + 1]);
        //out[i] += 0.0; //signal[i];
        //out[i] += signal[i];
        //out[i] += *(signal + i + 1) + fraction * (*(signal + i) - *(signal + i + 1));
    }

    return 0;
}

//int naive_delay(ring_buffer *rb, float *out, float delay, int sensor_id) {
//  for (int i = 0; i < N_TAPS; i++) {
//    signal_s[i + N_SAMPLES] = 0.0;
//  }
//
//
//  float bandpass[N_TAPS] = {-0.00037195,  0.00142594,  0.00428146,  0.0081584 ,  0.01147065,
//        0.01117933,  0.00420916, -0.01035545, -0.02955026, -0.04657337,
//       -0.05308094, -0.04297294, -0.01589055,  0.02148855,  0.05734645,
//        0.07923551,  0.07923551,  0.05734645,  0.02148855, -0.01589055,
//       -0.04297294, -0.05308094, -0.04657337, -0.02955026, -0.01035545,
//        0.00420916,  0.01117933,  0.01147065,  0.0081584 ,  0.00428146,
//        0.00142594, -0.00037195};
//
//
//
//
//
//
//  //std::copy(std::begin(signal_s) + N_TAPS, std::begin(signal_s) + N_SAMPLES + N_TAPS, out);
//  //std::fill(std::begin(signal_s), std::begin(signal_s)+N_TAPS, 0);
//
//  for (int i = 0; i < N_SAMPLES; i++)
//  {
//    out[i] = 0.0;
//    for (int k = 0; k < N_TAPS; k++) {
//      out[i] += bandpass[k] * signal_s[i + k];
//    }
//  }
//
//
//  return 0;
//}

int delay_(float *buffer, float *out, float delay, int sensor_id) {
    double _offset;
    float fraction;

    fraction = (float) modf((double) delay, &_offset);

    int offset = (int) _offset;

    int mic_idx = sensor_id * N_SAMPLES;

    for (int i = N_OFFSET - offset; i < N_SAMPLES; i++) {
        out[i] += buffer[mic_idx + i + 1] + fraction * (buffer[mic_idx + i] - buffer[mic_idx + i + 1]);

        //out[i] += signal[current] + fraction * (signal[prev] - signal[current]);
    }

    return 0;
}

#endif
