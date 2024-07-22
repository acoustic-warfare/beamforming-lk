#ifndef DELAY_H
#define DELAY_H

#include <algorithm>
#include <cmath>

#include "config.h"
//#include "filter.h"

#ifdef __AVX2__
#include <immintrin.h>
#endif

#define SAFETY_CHECK 0

void delay(float *out, const float *signal, const float fraction);

void delay_corrected(float *out, const float *signal, const float fraction, const float correction);

void delay_filtered(float *out, const float *signal, const float fraction, const float correction, int mode);

void delay_naive(float *out, const float *signal, const float fraction);

#ifdef __AVX2__
void delay_vectorized(float *out, const float *signal, const float fraction);
#endif


#endif
