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

/**
 * @brief Applies a fractional delay to the input signal and stores the result in the output buffer.
 * @param out Output buffer to store the delayed signal.
 * @param signal Input signal to be delayed.
 * @param fraction Fractional delay to be applied.
 */
void delay(float *out, const float *signal, const float fraction);

/**
 * @brief Applies a corrected fractional delay to the input signal and stores the result in the output buffer.
 * @param out Output buffer to store the delayed signal.
 * @param signal Input signal to be delayed.
 * @param fraction Fractional delay to be applied.
 * @param correction Correction factor for the delay.
 */
void delay_corrected(float *out, const float *signal, const float fraction, const float correction);

/**
 * @brief Applies a filtered fractional delay to the input signal and stores the result in the output buffer.
 * @param out Output buffer to store the delayed signal.
 * @param signal Input signal to be delayed.
 * @param fraction Fractional delay to be applied.
 * @param correction Correction factor for the delay.
 * @param mode Mode to specify the type of filtering.
 */
void delay_filtered(float *out, const float *signal, const float fraction, const float correction, int mode);

/**
 * @brief Applies a naive fractional delay to the input signal and stores the result in the output buffer.
 * @param out Output buffer to store the delayed signal.
 * @param signal Input signal to be delayed.
 * @param fraction Fractional delay to be applied.
 */
void delay_naive(float *out, const float *signal, const float fraction);

#ifdef __AVX2__
/**
 * @brief Applies a vectorized fractional delay to the input signal using AVX instructions and stores the result in the output buffer.
 * @param out Output buffer to store the delayed signal.
 * @param signal Input signal to be delayed.
 * @param fraction Fractional delay to be applied.
 */
void delay_vectorized(float *out, const float *signal, const float fraction);
#endif

#endif //BEAMFORMER_DELAY_H
