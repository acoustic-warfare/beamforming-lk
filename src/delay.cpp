#include "delay.h"
#include <string.h>

#if 0
void delay(float *out, const float *signal, const float fraction) {
    for (int i = 0; i < N_SAMPLES; i++) {
        out[i] += signal[i + 1] + fraction * (signal[i] - signal[i + 1]);
    }
}

#elif 0


void delay(float *out, const float *signal, const float fraction) {
    for (int i = 0; i < N_SAMPLES; i++) {
        out[i] += signal[i + 1] + fraction * (signal[i] - signal[i + 1]);
    }
}

#elif 1

#define K 8
//float h[K] = {0.82513184, -3.30052736, 4.95079104, -3.30052736, 0.82513184};
float h[K] = {0.0267, 0.1238, 0.3025, 0.3025, 0.1238, 0.0267, -0.0200, 0.0};

//float h[K + 1] = {0.000001045799686788, 0.000002017579915724, 0.000000800908686074, -0.000004281288000602, -0.000009732441567983, -0.000007250939315188, 0.000004092186395343, 0.000007365756230637, -0.000018464289827503, -0.000064734259950475, -0.000079263024946202, -0.000004183852478728, 0.000159288397643241, 0.000323141333054213, 0.000352469484563562, 0.000130059116908686, -0.000423309113457044, -0.001387723524125118, -0.002888174421242549, -0.003437530001174163, -0.006041725618878423, -0.012558287945196277, -0.022827317579910699, -0.029278234926486211, -0.023292499414214581, -0.013498822318471371, -0.026974869436859197, -0.074873160009987588, -0.117000756114553878, -0.082796587256388721, 0.052531939863417101, 0.217696477130204402, 0.292516420440647629, 0.217696477130202626, 0.052531939863417143, -0.082796587256388970, -0.117000756114553794, -0.074873160009987422, -0.026974869436859138, -0.013498822318471163, -0.023292499414214549, -0.029278234926486187, -0.022827317579910786, -0.012558287945196336, -0.006041725618878438, -0.003437530001174179, -0.002888174421242552, -0.001387723524125124, -0.000423309113457045, 0.000130059116908676, 0.000352469484563535, 0.000323141333054190, 0.000159288397643239, -0.000004183852478730, -0.000079263024946219, -0.000064734259950473, -0.000018464289827506, 0.000007365756230645, 0.000004092186395340, -0.000007250939315211, -0.000009732441567994, -0.000004281288000609, 0.000000800908686064, 0.000002017579915715, 0.000001045799686794};

//void delay(float *out, const float *signal, const float fraction) {
//    float a[K + 1] = {0.0};
//    for (int i = 0; i < N_SAMPLES; i++) {
//        float _out = out[i];
//        for (int j = 0; j < K; j++) {
//            _out += a[j] * h[j];
//        }
//        for (int j = 0; j < K; j++) {
//            a[j] = a[j + 1];
//        }
//        a[K] = signal[i + 1] + fraction * (signal[i] - signal[i + 1]);
//        out[i] = _out;
//    }
//}

#include <immintrin.h>

#define AVX_VECTOR 8

void
delay(float *out, const float *signal, const float fraction) {
    //float a[9] = {0.0f};              // K is 8, so a size of 9 to hold K+1 elements
    //__m256 h_vec = _mm256_loadu_ps(h);// Load all h elements into a single vector

    __m256 fraction_vec = _mm256_set1_ps(fraction);
    //__m256 buffer = _mm256_setzero_ps();

    for (int i = 0; i < N_SAMPLES; i+=AVX_VECTOR) {
        __m256 out_vec = _mm256_loadu_ps(out + i);
        __m256 current_vec = _mm256_loadu_ps(signal + i);
        __m256 next_vec = _mm256_loadu_ps(signal + i + 1);

        //__m256 acc = _mm256_sub_ps(current_vec, next_vec);
        //__m256 result_vec = _mm256_fmadd_ps(fraction_vec, _mm256_sub_ps(current_vec, next_vec), next_vec);

        _mm256_storeu_ps(&out[i], _mm256_add_ps(out_vec, _mm256_fmadd_ps(fraction_vec, _mm256_sub_ps(current_vec, next_vec), next_vec)));


#if 0
        _mm256_fmadd_ps()

        _mm256_fmsubadd_ps
        // Load 8 elements from a into an AVX register
        __m256 a_vec = _mm256_loadu_ps(a);

        // Perform the vectorized multiply-add operation: out[i] += a[j] * h[j] for j in 0..7
        __m256 out_vec = _mm256_mul_ps(a_vec, h_vec);

        // Horizontally sum the elements of out_vec to get the final result for out[i]
        out[i] = _mm256_reduce_add_ps(out_vec);

        // Shift elements in the array 'a' to the left
        for (int j = 0; j < 8; j++) {
            a[j] = a[j + 1];
        }

        // Compute the new a[8]
        a[8] = signal[i + 1] + fraction * (signal[i] - signal[i + 1]);
#endif
    }
}

#else

void delay_corrected(float *out, const float *signal, const float fraction, const float correction = 1.0) {
    for (int i = 0; i < N_SAMPLES; i++) {
        out[i] += correction * (signal[i + 1] + fraction * (signal[i] - signal[i + 1]));
    }
}

void delay_filtered(float *out, const float *signal, const float fraction,  int mode, const float correction){
    int delay_integer = std::round(fraction * 100);
    float filter_coos [55];

    // Mode is given as a positive integer. 1-7. Right now the modes implemented are 
    // 1-4. 5,6,7 are pretty bad low pass. 


    // delay is (ntaps - 1) / 2 = 27 samples. Considering ntaps = 55 and the first zero is there for symmetry and for it to be a multiple of 8. 


    for (int i = 0; i < 56; ++i){
       filter_coos[i] = filter_coeffs[mode - 1][delay_integer][i] * correction;
    }


    for(int n = 56; n < N_SAMPLES; ++n){
        for(int c = -28; c < 27; ++c){
            out[n] +=  filter_coos[c + 28] * signal[n-c];
        }
    }

}


#ifndef __AVX2__

//void delay(float *out, const float *signal, const float fraction) {
//    for (int i = 0; i < N_SAMPLES; i++) {
//        out[i] += signal[i + 1] + fraction * (signal[i] - signal[i + 1]);
//    }
//}

#else

#define ALIGNMENT 32
#define AVX_SIMD_LENGTH 8

void _delay(float *out, const float *signal, const float fraction) {
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

#endif
