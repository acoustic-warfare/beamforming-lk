/** @file delay.cpp
 * @author Irreq, Melker
*/

#include "delay.h"
#include <string.h>

#if __AVX2__

// Kernel for delay, waiting for implementation
#define K 8
float h[K] = {0.0267, 0.1238, 0.3025, 0.3025, 0.1238, 0.0267, -0.0200, 0.0};

#define AVX_VECTOR 8

void delay(float *out, const float *signal, const float fraction) {
    __m256 fraction_vec = _mm256_set1_ps(fraction);

    for (int i = 0; i < N_SAMPLES; i += AVX_VECTOR) {
        __m256 out_vec = _mm256_loadu_ps(out + i);
        __m256 current_vec = _mm256_loadu_ps(signal + i);
        __m256 next_vec = _mm256_loadu_ps(signal + i + 1);

        _mm256_storeu_ps(&out[i], _mm256_add_ps(out_vec, _mm256_fmadd_ps(fraction_vec, _mm256_sub_ps(current_vec, next_vec), next_vec)));
    }
}

#elif USE_FILTER

#define COEFF_SIZE 8    // Filter coefficient size (assuming it is 8)
void delay(float *out, const float *signal, const float fraction) {
    float get_filter = fraction * 100.0f + 0.5f;
    int delay_int = (int) get_filter;

    for (int n = 0; n < N_SAMPLES; ++n) {
        for (int i = 0; i < COEFF_SIZE; ++i) {
            out[n] += filter_coeffs[delay_int][i] * signal[n + i];
        }
    }
}

#else

void delay(float *out, const float *signal, const float fraction) {
    for (int i = 0; i < N_SAMPLES; i++) {
        out[i] += signal[i + 1] + fraction * (signal[i] - signal[i + 1]);
    }
}

#endif
