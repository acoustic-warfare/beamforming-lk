#pragma GCC optimize("O3")

#include "config.h"

#include "antenna.h"
#include "delay.h"
#include "options.h"
#include "pipeline.h"
#include "ring_buffer.h"

#if AUDIO
#include "RtAudio.h"
#endif

#include <Eigen/Dense>
#include <WaraPSClient.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <signal.h>
#include <thread>

using namespace Eigen;
using namespace cv;

// make && sudo chrt -f 98 ./beamformer

// #define VALID_SENSOR(i) ((128 <= i) && (128 + 64 > i))
#define VALID_SENSOR(i) (64 <= i) && (i < 128)
bool canPlot = false;

/**
 * @brief Calculate delays for different angles beforehand
 *
 * @param flat_delays delays to use
 * @param antenna antenna structure
 * @param fov field of view
 * @param resolution_x width resolution
 * @param resolution_y height resolution
 */
void compute_scanning_window(float *flat_delays, const Antenna &antenna,
                             float fov, int resolution_x, int resolution_y) {

    float half_x = (float) (resolution_x) / 2 - 0.5;
    float half_y = (float) (resolution_y) / 2 - 0.5;
    int k = 0;
    for (int x = 0; x < resolution_x; x++) {
        for (int y = 0; y < resolution_y; y++) {

            // Imagine dome in spherical coordinates on the XY-plane with Z being height
            float xo = (float) (x - half_x) / (resolution_x);
            float yo = (float) (y - half_y) / (resolution_y);
            float level = sqrt(xo * xo + yo * yo) / 1;
            level = sqrt(1 - level * level);
            Position point(xo, yo, level);
            // cout << point << endl;

            VectorXf tmp_delays = steering_vector(antenna, point);
            int i = 0;
            for (float del: tmp_delays) {
                // cout << del << endl;
                flat_delays[k * N_SENSORS + i] = del;
                i++;
            }

            k++;
        }
    }
}

/**
 * @brief Convert multiple input streams into single level by delay
 *
 * @param t_id [TODO:parameter]
 * @param task pool partition
 * @param flat_delays delays to use
 * @param rb ring buffer to use
 * @return power level
 */
float miso(int t_id, int task, float *flat_delays, ring_buffer &rb) {
    float out[N_SAMPLES] = {0.0};
    int n = 0;
    for (int s = 0; s < N_SENSORS; s++) {

        // if (!((s == 64) || (s == 64 + 8) || (s == 127 - 16) || (s == 127))) {
        //   continue;
        // }

        if (VALID_SENSOR(s)) {
            float del = flat_delays[s - 64];
            // float del = flat_delays[s - 128];
            //  cout << s << " ";
            naive_delay(&rb, &out[0], del, s);
            n++;
        }
    }

    // cout << endl;

    float power = 0.f;
    for (int p = 0; p < n; p++) {

        float val = out[p] / (float) n;

        power += powf(val, 2);
    }

    return power / (float) n;
}

///**
// * @brief Convert multiple input streams into single level by delay
// *
// * @param t_id [TODO:parameter]
// * @param task pool partition
// * @param flat_delays delays to use
// * @param rb ring buffer to use
// * @return power level
// */
// float miso(int t_id, int task, float *flat_delays, float *rb) {
//  float out[N_SAMPLES] = {0.0};
//  int n = 0;
//  for (int s = 0; s < N_SENSORS; s++) {
//
//    //if (!((s == 64) || (s == 64 + 8) || (s == 127 - 16) || (s == 127))) {
//    //  continue;
//    //}
//
//    if (VALID_SENSOR(s)) {
//      float del = flat_delays[s - 64];
//      //float del = flat_delays[s - 128];
//      // cout << s << " ";
//      naive_delay(&rb, &out[0], del, s);
//      n++;
//    }
//  }
//
//  // cout << endl;
//
//
//  float power = 0.f;
//  for (int p = 0; p < n; p++) {
//
//    float val = out[p] / (float)n;
//
//    power += powf(val, 2);
//  }
//
//  return power / (float)n;
//}

Mat noiseMatrix(Y_RES, X_RES, CV_8UC1);

// If Audio playback when streaming
#if AUDIO

RtAudio audio;
int play = 1;
std::thread *producer;
std::vector<float> audioBuffer(N_SAMPLES * 2, 0.0);

/**
 * @brief Producer for audio on pipeline
 *
 * @param pipeline Pipeline
 */
void audio_producer(Pipeline &pipeline) {

    ring_buffer &rb = pipeline.getRingBuffer();

    float out[N_SAMPLES] = {0.0};

    while (pipeline.isRunning()) {

        for (int i = 0; i < N_SAMPLES; i++) {
            out[i] /= 64.f;

            out[i] *= 100.f;
            audioBuffer[i * 2] = out[i];
            audioBuffer[i * 2 + 1] = out[i];
            out[i] = 0.0;
        }
        play = 0;

        pipeline.barrier();

        // for (int s = 0; s < N_SENSORS; s++) {
        //
        //   if (VALID_SENSOR(s)) {
        //     // cout << s << " ";
        //     naive_delay(&rb, &out[0], 0.0, s);
        //   }
        // }

        naive_delay(&rb, &out[0], 0.0, 140);

        // for (int i = 0; i < N_SAMPLES; i++) {
        //   audioBuffer[i * 2] = rb.data[140][rb.index + i];
        //   audioBuffer[i * 2 + 1] = rb.data[140][rb.index + i];
        // }

        // cout << "run" << endl;

        // memcpy(&yrb.data[140][rb.index], &audioBuffer[0],
        //        N_SAMPLES * sizeof(float));
    }
}

/**
 * @brief Callback for audio stream
 *
 * @param outputBuffer Speaker buffer
 * @param inputBuffer empty (Required by RtAudio API)
 * @param nBufferFrames number of frames to fill
 * @param streamTime duration
 * @param status status
 * @param userData the incoming data
 * @return OK
 */
int audioCallback(void *outputBuffer, void *inputBuffer,
                  unsigned int nBufferFrames, double streamTime,
                  RtAudioStreamStatus status, void *userData) {
    float *buffer = (float *) outputBuffer;

    // Copy samples from the sineBuffer to the output buffer for playback
    for (unsigned int i = 0; i < N_SAMPLES * 2; ++i) {
        if (!play) {
            *buffer++ = audioBuffer[i];
        } else {
            cout << "Underflow" << endl;
            *buffer++ = 0.0;
        }
    }

    play = 1;

    return 0;
}

/**
 * @brief Initiate Audio player for Pipeline
 *
 * @param pipeline the pipeline to follow
 * @return status
 */
int init_audio_playback(Pipeline &pipeline) {
    if (audio.getDeviceCount() < 1) {
        std::cout << "No audio devices found!" << std::endl;
        return EXIT_FAILURE;
    }

    RtAudio::StreamParameters parameters;
    parameters.deviceId = audio.getDefaultOutputDevice();
    parameters.nChannels = 2;// Stereo output

    try {
        unsigned int bufferFrames = N_SAMPLES;
        audio.openStream(&parameters, nullptr, RTAUDIO_FLOAT32, 44100.f,
                         &bufferFrames, &audioCallback);
        audio.startStream();

        producer = new std::thread(audio_producer, ref(pipeline));
    } catch (RtAudioErrorType &e) {
        // std::cout << "Error: " << e.getMessage() << std::endl;
        return 1;
    }

    return 0;
}

void stop_audio_playback() {
    // Start the separate thread for sine wave generation

    // Keep the program running

    // Stop the sine wave generation thread
    producer->join();

    // Stop and close the RtAudio stream
    audio.stopStream();
    audio.closeStream();
}

#endif

/**
 * Beamforming as fast as possible on top of pipeline
 */
void naive_seeker_old(Pipeline &pipeline) {

    Antenna antenna = create_antenna(Position(0, 0, 0), COLUMNS, ROWS, DISTANCE);

    float flat_delays[X_RES * Y_RES * N_SENSORS];

    compute_scanning_window(&flat_delays[0], antenna, FOV, X_RES, Y_RES);

#if 0
  int i = 0;
  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 8; x++) {
      cout << flat_delays[i++] << " ";
    }

    cout << "\n";
  }

  cout << endl;

  std::cout << flat_delays << std::endl;

#endif

    int max = X_RES * Y_RES;

    float image[X_RES * Y_RES];

    int pixel_index = 0;

    // return;

    int newData;
    // ring_buffer rb;
    //
    // Initialize random seed
    srand(static_cast<unsigned int>(0));
    // Create a 100x100 matrix to display noise
    // Mat noiseMatrix(Y_RES, X_RES, CV_8UC1);
    float decay = 2.0;
    float avg_min = 1.0;

    float maxVal = 1.0;
    float maxDecay = 1.0, minDecay = 1.0;
    maxDecay = 0.01;

    float div = 0.07;
    float threshold = 3e-8;

    float power = threshold * 0.9;

    while (pipeline.isRunning()) {

        // Wait for incoming data
        pipeline.barrier();

        // This loop may run until new data has been produced, meaning its up to
        // the machine to run as fast as possible
        newData = pipeline.mostRecent();
        ring_buffer &rb = pipeline.getRingBuffer();

        int i = 0;
        float mean = 0.0;

        int xi, yi = 0;
        float alpha = 1.0 / (float) (X_RES * Y_RES + 4);
        alpha = 0.3;

        float heatmap_data[X_RES * Y_RES];

        // maxDecay = 0.0;
        maxVal = 0.0;
        float avgPower = 0.0;

        // Repeat until new data or abort if new data arrives
        while ((pipeline.mostRecent() == newData) && (i < max)) {
            int task = pixel_index * N_SENSORS;

            xi = pixel_index % X_RES;
            yi = pixel_index / X_RES;

            // Get power level from direction
            float val = miso(0, pixel_index, &flat_delays[task], rb);

            // TODO normalize value :(

            if (val > avgPower) {
                avgPower = val;
            }

            // minDecay = alpha * std::min(val, minDecay) + (1.0 - alpha) * minDecay;
            // maxDecay = alpha * std::max(val, maxDecay) + (1.0 - alpha) * maxDecay;

            // cout << maxDecay << endl;
            //
            // val /= maxDecay * 1.1;
            // val /= maxDecay;
            //

            val *= 1e9;
            val = log(val);

            if (val > maxVal) {
                maxVal = val;
            }

            val /= maxDecay;

            val = powf(val, 13);

            // val /= 20.f;

            // cout << maxDecay << endl;

            // //
            // // val /= div;
            // // val = powf(val, 10);
            //
            // // cout << val << endl;
            // // val *= 3e7;
            // //
            //
            // // avg_min = alpha * val + (1 - alpha) * avg_min;
            // // cout << avg_min << endl;
            // // val /= avg_min;
            //
            // // val *= 1e9;
            // // val = log(val);
            // // val /= 10;
            //
            // // val = 1 / (1 + val);
            //

            //
            // if (decay > threshold) {
            //   // val /= 10 * decay;
            //   // val /= maxDecay;
            //   ;
            //   // cout << decay << endl;
            // }
            //
            // if (val > 1.0) {
            //   cout << "Over: " << val << endl;
            //   val = 1;
            // }
            //
            decay = alpha * val + (1 - alpha) * decay;

            if (power < threshold) {
                val = 0.0;
            } else if (val > 1.0) {
                // cout << val << endl;
                val = 1;
            } else if (val < 0.0) {
                val = 0;
                // cout << "Negative value" << endl;
            }

            // Paint pixel
            noiseMatrix.at<uchar>(yi, xi) = (uchar) (val * 255);
            canPlot = true;

            pixel_index++;
            pixel_index %= X_RES * Y_RES;

            i++;
        }

        // cout << maxDecay << " " << maxVal << endl;
        // maxDecay *= 0.98;
        // float tp = maxDecay * 0.97;
        // maxDecay = alpha * std::max(maxVal, tp) + (1.0 - alpha) * maxDecay;
        //
        if (maxVal > maxDecay) {
            maxDecay = maxVal;
        } else {
            maxDecay = alpha * maxVal + (1.0 - alpha) * maxDecay;
        }

        if (avgPower > power) {
            power = avgPower;
        } else {
            power = alpha * avgPower + (1.0 - alpha) * power;
        }

        // maxDecay *= 1.01;

        // maxDecay /= 2.0; // 1.0 - alpha;
        // maxDecay = alpha * std::max(maxVal, maxDecay) + (1.0 - alpha) *
        // maxDecay; maxVal /= 2.0; cout << maxVal << endl;
        // maxVal *= 0.99;
    }
}

/**
 * Beamforming as fast as possible on top of pipeline
 */
void naive_seeker(Pipeline &pipeline, BeamformingOptions &options) {

    Antenna antenna = create_antenna(Position(0, 0, 0), COLUMNS, ROWS, DISTANCE);

    float flat_delays[X_RES * Y_RES * N_SENSORS];

    compute_scanning_window(&flat_delays[0], antenna, FOV, X_RES, Y_RES);

    int max = X_RES * Y_RES;

    float image[X_RES * Y_RES];

    int pixel_index = 0;

    // return;

    int newData;
    float power;
    float threshold = 3e-8;

    float norm = 1 / 1e-05;
    // ring_buffer rb;
    //
    // Initialize random seed
    srand(static_cast<unsigned int>(0));

    float maxVal = 1.0;

    while (pipeline.isRunning()) {

        // Wait for incoming data
        pipeline.barrier();

        // This loop may run until new data has been produced, meaning its up to
        // the machine to run as fast as possible
        newData = pipeline.mostRecent();
        ring_buffer &rb = pipeline.getRingBuffer();
        float maxVal = 0.0;

        int i = 0;
        float mean = 0.0;

        int xi, yi = 0;
        float alpha = 1.0 / (float) (X_RES * Y_RES + 4);
        alpha = 0.02;

        float heatmap_data[X_RES * Y_RES];

        // maxDecay = 0.0;

        float avgPower = 0.0;

        // Repeat until new data or abort if new data arrives
        // while ((pipeline.mostRecent() == newData) && (i < max)) {

        while ((i < max)) {
            int task = pixel_index * N_SENSORS;

            xi = pixel_index % X_RES;
            yi = pixel_index / X_RES;

            // Get power level from direction
            float val = miso(0, pixel_index, &flat_delays[task], rb);

            if (val > maxVal) {
                maxVal = val;
            }

            // power = val * 1e5;

            // power = val * norm * 0.9f + 1.0;
            power = val + 1.0f;
            power = powf(power, 15);
            // power *= 1e9f;

            power = log(power) * 0.1f;

            power = power * norm * 0.9f;

            if (power < 0.2) {
                power = 0.0f;
            } else if (power > 1.0) {
                norm *= 0.95;
                // cout << "Bigger value" << endl;
                power = 1.0f;
            } else if (power < 0.0) {
                power = 0.0f;
                // cout << "Negative value" << endl;
            }

            // Paint pixel
            noiseMatrix.at<uchar>(yi, xi) = (uchar) (power * 255);
            // canPlot = true;

            pixel_index++;
            pixel_index %= X_RES * Y_RES;

            i++;
        }

        canPlot = true;

        norm = (1 - alpha) * norm + alpha * (1 / (maxVal));

        // norm = (1/maxVal) * 1.1f;

        // cout << maxVal << endl;
    }
}

Pipeline pipeline = Pipeline();

void sig_handler(int sig) {
    // Set the stop_processing flag to terminate worker threads gracefully

    pipeline.disconnect();
}

int main() {
    WaraPSClient client = WaraPSClient("test", "mqtt://localhost:25565");
    BeamformingOptions options;

    client.set_command_callback("focus_bf", [&](const nlohmann::json &payload) {
        float theta = payload["theta"];
        float phi = payload["phi"];
        float duration = payload.contains("duration") ? (float)payload["duration"] : 5.0f;

        cout << "Theta: " << theta << "\nPhi: " << phi << endl;
        client.publish_message("exec/response", string("Focusing beamformer for " + to_string(duration)));
    });

    thread client_thread = client.start();

    // Connect to UDP stream
    pipeline.connect();

    // Start beamforming thread
    thread worker(naive_seeker, ref(pipeline), ref(options);

    // Initiate background image
    noiseMatrix.setTo(Scalar(0));

    // Create a window to display the noise matrix
    cv::namedWindow("Noise Matrix", WINDOW_NORMAL);
    cv::resizeWindow("Noise Matrix", 800, 800);
    int res = 10;
    Mat previous(Y_RES, X_RES, CV_8UC1);
    previous.setTo(Scalar(0));
    applyColorMap(previous, previous, COLORMAP_JET);
    resize(previous, previous, Size(), res, res, INTER_LINEAR);

    cout << "Finished preparing beamformer" << endl;
    cout << "Starting to plot" << endl;

    while (pipeline.isRunning()) {
        if (canPlot) {
            canPlot = false;
            Mat coloredMatrix;
            // Blur the image with 3x3 Gaussian kernel
            // Mat image_blurred_with_3x3_kernel;
            GaussianBlur(noiseMatrix, coloredMatrix, Size(3, 3), 0);
            GaussianBlur(coloredMatrix, coloredMatrix, Size(3, 3), 0);
            applyColorMap(noiseMatrix, coloredMatrix, COLORMAP_JET);

            cv::resize(coloredMatrix, coloredMatrix, Size(), res, res, INTER_LINEAR);
            addWeighted(coloredMatrix, 0.99, previous, 0.01, 0, coloredMatrix);
            previous = coloredMatrix;
            cv::imshow("Noise Matrix", coloredMatrix);
        }

        if (!client.running() || waitKey(1) == 27) {
            break;
        }
    }

    pipeline.save_pipeline("pipeline.bin");
    pipeline.disconnect();

#if AUDIO
    stop_audio_playback();
#endif
    destroyAllWindows();
    client_thread.join();
    worker.join();
}
