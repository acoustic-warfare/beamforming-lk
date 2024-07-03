#include "delay.h"

#ifndef __AVX2__

void delay(float *out, const float *signal, const float fraction) {
    for (int i = 0; i < N_SAMPLES; i++) {
        out[i] += signal[i + 1] + fraction * (signal[i] - signal[i + 1]);
    }
}

#else

#define ALIGNMENT 32
#define AVX_SIMD_LENGTH 8

void delay(float *out, const float *signal, const float fraction) {
    __m256 fraction_block = _mm256_broadcast_ss(&fraction);

    __m256 accumulator = _mm256_setzero_ps();
    for (int i = 0; i < N_SAMPLES; i += AVX_SIMD_LENGTH) {
        __m256 signal_current_block = _mm256_loadu_ps(signal + i);
        __m256 signal_next_block = _mm256_loadu_ps(signal + i + 1);

        accumulator = _mm256_sub_ps(signal_current_block, signal_next_block);


        signal_next_block = _mm256_fmadd_ps(fraction_block, accumulator, signal_next_block);

        __m256 acc = _mm256_loadu_ps(out + i);
        acc = _mm256_add_ps(acc, signal_next_block);

        //__m256 accumulator = _mm256_setzero_ps();
        //accumulator = _mm256_fmadd_ps(signal_block, fraction_block, accumulator);
        _mm256_storeu_ps(out + i, acc);
    }

    //__m256 data1_block __attribute__((aligned(ALIGNMENT)));
    //__m256 data2_block __attribute__((aligned(ALIGNMENT)));
    //__m256 accumulator __attribute__((aligned(ALIGNMENT)));
    //
    //__m256 fraction_block __attribute__((aligned(ALIGNMENT))) =_mm256_broadcast_ss(fraction);
    //for (int i = 0; i < N_SAMPLES; i += AVX_SIMD_LENGTH) {
    //    accumulator = _mm256_setzero_ps();
    //
    //    data1_block = _mm256_loadu_ps(signal + i * sizeof(float));
    //    accumulator = _mm256_fmadd_ps(fraction_block, data1_block, accumulator);
    //}
}

#endif


#if 0

/*
Vectorized convolution for AVX2 with single accumulator without reset
*/
//void convolve_delay_vectorized_add(float *signal, float *h, float *out)
//{
//    __m256 data_block __attribute__((aligned(ALIGNMENT)));
//    __m256 kernel_block __attribute__((aligned(ALIGNMENT)));
//
//    float *padded = (float *)_mm_malloc((N_SAMPLES + N_TAPS) * sizeof(float), ALIGNMENT);
//
//    // Initialize the padded array with zeros
//    memset(padded, 0, (N_SAMPLES + N_TAPS) * sizeof(float));
//    // Copy the original array to the padded array
//    memcpy(padded + OFFSET, signal, N_SAMPLES * sizeof(float));
//
//    float *kernel = (float *)_mm_malloc((N_TAPS) * sizeof(float), ALIGNMENT); //(float *)aligned_alloc(ALIGNMENT, (N_TAPS) * sizeof(float));
//    memcpy(kernel, h, N_TAPS * sizeof(float));
//
//    __m256 accumulator __attribute__((aligned(ALIGNMENT)));
//
//    for (int i = 0; i < N_SAMPLES; i++)
//    {
//        accumulator = _mm256_setzero_ps();
//
//        for (int k = 0; k < N_TAPS; k += AVX_SIMD_LENGTH)
//        {
//            data_block = _mm256_loadu_ps(padded + i + k);
//            kernel_block = _mm256_load_ps(kernel + k);
//            accumulator = _mm256_fmadd_ps(data_block, kernel_block, accumulator);
//        }
//
//        out[i] += sum8(accumulator);
//    }
//
//    // Free the allocated memory
//    _mm_free(padded);
//    _mm_free(kernel);
//}

#include "delay.h"

// C++ program to find out execution time of
// of functions
#include <algorithm>
#include <chrono>
#include <iostream>
#include <vector>
using namespace std;
using namespace std::chrono;

#define N 256 * 256
 
// For demonstration purpose, we will fill up
// a vector with random integers and then sort
// them using sort function. We fill record
// and print the time required by sort function
int main()
{

    float signal[N_SAMPLES];
    float out_slow[N_SAMPLES] = {0.0};
    float out_fast[N_SAMPLES] = {0.0};
 
    // Fill up the vector
    for (int i = 0; i < N_SAMPLES; i++) {
        signal[i] = (float)i;
    }
 
    // Get starting timepoint
    auto start = high_resolution_clock::now();

    for (int i = 0; i < N; i++) {
        // Call the function, here sort()
        delay_naive(&out_slow[0], &signal[0], 1.5);
    }
 
    // Get ending timepoint
    auto stop = high_resolution_clock::now();
 
    // Get duration. Substart timepoints to 
    // get duration. To cast it to proper unit
    // use duration cast method
    auto duration_naive = duration_cast<nanoseconds>(stop - start);

    cout << "Time taken by slow delay: "
         << duration_naive.count() << " nanoseconds" << endl;

    // Get starting timepoint
    start = high_resolution_clock::now();

    for (int i = 0; i < N; i++) {
        // Call the function, here sort()
        delay_vectorized(&out_fast[0], &signal[0], 1.5);
    }
 
    // Get ending timepoint
    stop = high_resolution_clock::now();
 
    // Get duration. Substart timepoints to 
    // get duration. To cast it to proper unit
    // use duration cast method
    auto duration_vectorized = duration_cast<nanoseconds>(stop - start);

    cout << "Time taken by fast delay: "
         << duration_vectorized.count() << " nanoseconds" << endl;
    

    for (int i = 0; i < 20; i++) {
        std::cout << "(" << out_slow[i] << ", " << out_fast[i] << ") <- " << signal[i] << std::endl;
    }
 
    

    
 
    return 0;
}

#endif