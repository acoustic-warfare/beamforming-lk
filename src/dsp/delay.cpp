/** @file delay.cpp
 * @author Irreq, Melker
*/

#include "delay.h"
#include <string.h>

#define USE_AVX 1

#define K 8
float h[K] = {0.0267, 0.1238, 0.3025, 0.3025, 0.1238, 0.0267, -0.0200, 0.0};

#if USE_AVX

#include <immintrin.h>

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

#else

void delay(float *out, const float *signal, const float fraction) {
    for (int i = 0; i < N_SAMPLES; i++) {
        out[i] += signal[i + 1] + fraction * (signal[i] - signal[i + 1]);
    }
}

#endif
