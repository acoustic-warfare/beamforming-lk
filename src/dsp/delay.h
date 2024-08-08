/** @file delay.h
 * @author Irreq, Melker
 * @brief Provides functions for applying various types of fractional delays to input signals.
 */

#ifndef BEAMFORMER_DELAY_H
#define BEAMFORMER_DELAY_H

#include <algorithm>
#include <cmath>

#include "pipeline.h"

#ifdef __AVX2__
#include <immintrin.h>
#endif

#define SAFETY_CHECK 0
#define USE_FILTER 1

#if USE_FILTER
#include "filter.h"
#endif

/**
 * @brief Applies a fractional delay to the input signal and stores the result in the output buffer.
 * @param out Output buffer to store the delayed signal.
 * @param signal Input signal to be delayed.
 * @param fraction Fractional delay to be applied.
 */
void delay(float *out, const float *signal, const float fraction);

#endif //BEAMFORMER_DELAY_H
