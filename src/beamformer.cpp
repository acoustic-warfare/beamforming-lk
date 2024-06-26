

#include "config.h"

#include "antenna.h"
#include "delay.h"
#include "options.h"
#include "pipeline.h"


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

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <atomic>

#define VALID_SENSOR(i) (64 <= i) && (i < 128)


/**
Main application for running beamformer program

You may try to use:

sudo chrt -f 98 ./beamformer

for better performace by altering the real-time scheduling attributes of the program.

 */


std::atomic_int canPlot = 0;

Pipeline *pipeline;



// Intermediate heatmap used for beamforming (8-bit)
cv::Mat magnitudeHeatmap(Y_RES, X_RES, CV_8UC1);

/**
 * @brief Calculate delays for different angles beforehand 
 *
 * @param fractional_delays delays to use
 * @param antenna antenna structure 
 * @param fov field of view 
 * @param resolution_x width resolution 
 * @param resolution_y height resolution
 */
void compute_scanning_window(int *offset_delays, float *fractional_delays, const Antenna &antenna,
                             float fov, int resolution_x, int resolution_y) {

  float half_x = (float)(resolution_x) / 2 - 0.5;
  float half_y = (float)(resolution_y) / 2 - 0.5;
  int k = 0;
  for (int x = 0; x < resolution_x; x++) {
    for (int y = 0; y < resolution_y; y++) {

      // Imagine dome in spherical coordinates on the XY-plane with Z being height
      float xo = (float)(x - half_x) / (resolution_x);
      float yo = (float)(y - half_y) / (resolution_y);
      float level = sqrt(xo * xo + yo * yo) / 1;
      level = sqrt(1 - level * level);
      Position point(xo, yo, level);
      // cout << point << endl;

      VectorXf tmp_delays = steering_vector(antenna, point);
      int i = 0;
      for (float del : tmp_delays) {
        double _offset;
        float fraction;

        fraction = (float)modf((double)del, &_offset);

        int offset = N_SAMPLES - (int)_offset;
        // cout << del << endl;
        fractional_delays[k * N_SENSORS + i] = fraction;
        offset_delays[k * N_SENSORS + i] = offset;
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
 * @param fractional_delays delays to use
 * @param rb ring buffer to use 
 * @return power level
 */
float miso(int t_id, int task, int *offset_delays, float *fractional_delays, Streams *streams) {
  float out[N_SAMPLES] = {0.0};
  int n = 0;
  for (int s = 0; s < N_SENSORS; s++) {

    //if (!((s == 64) || (s == 64 + 8) || (s == 127 - 16) || (s == 127))) {
    //  continue;
    //}

    if (VALID_SENSOR(s)) {
      float fraction = fractional_delays[s - 64];
      int offset = offset_delays[s - 64];

      float *signal = (float*)((char*)streams->buffers[s] + streams->position + offset * sizeof(float));

      for (int i = 0; i < N_SAMPLES; i++) {
        out[i] += signal[i + 1] + fraction * (signal[i] - signal[i + 1]);
      }

      n++;
    }
  }


  float power = 0.f;
  float norm = 1 / (float)n;
  for (int p = 0; p < N_SAMPLES; p++) {

    power += powf(out[p] * norm, 2);
  }

  return power / (float)N_SAMPLES;
}



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
  float *buffer = (float *)outputBuffer;

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
  parameters.nChannels = 2; // Stereo output

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
void naive_seeker(Pipeline *pipeline) {

  Antenna antenna = create_antenna(Position(0, 0, 0), COLUMNS, ROWS, DISTANCE);

  float fractional_delays[X_RES * Y_RES * N_SENSORS];
  int offset_delays[X_RES * Y_RES * N_SENSORS];

  compute_scanning_window(&offset_delays[0], &fractional_delays[0], antenna, FOV, X_RES, Y_RES);


  int max = X_RES * Y_RES;

  float image[X_RES * Y_RES];

  int pixel_index = 0;

  int newData;
  float power;
  float threshold = 3e-8;

  float norm = 1 / 1e-05;


  float maxVal = 1.0;

  Streams *streams = pipeline->getStreams();


  while (pipeline->isRunning()) {

    // Wait for incoming data
    pipeline->barrier();

    // This loop may run until new data has been produced, meaning its up to
    // the machine to run as fast as possible
    newData = pipeline->mostRecent();
    float maxVal = 0.0;


    int i = 0;
    float mean = 0.0;

    int xi, yi = 0;
    float alpha = 1.0 / (float)(X_RES * Y_RES);
    alpha = 0.02;

    float heatmap_data[X_RES * Y_RES];

    float avgPower = 0.0;

    // Repeat until new data or abort if new data arrives
  while ((pipeline->mostRecent() == newData) && (i < max)) {

      int task = pixel_index * N_SENSORS;

      xi = pixel_index % X_RES;
      yi = pixel_index / X_RES;

      // Get power level from direction
      float val = miso(0, pixel_index, &offset_delays[task], &fractional_delays[task], streams);

      if (val > maxVal) {
        maxVal = val;
      }

      //power = val * 1e5;

      //power = val * norm * 0.9f + 1.0;
      power = val + 1.0f;
      power = powf(power, 15);
      //power *= 1e9f;

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
      magnitudeHeatmap.at<uchar>(yi, xi) = (uchar)(power * 255);

      pixel_index++;
      pixel_index %= X_RES * Y_RES;

      i++;
    }

    canPlot = 1;

    norm = (1-alpha) * norm + alpha * (1/(maxVal));

    //norm = (1/maxVal) * 1.1f;

    //cout << maxVal << endl;

    
  }
}



void sig_handler(int sig) {
  // Set the stop_processing flag to terminate worker threads gracefully
  pipeline->disconnect();
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

  // Setup sigint i.e Ctrl-C
  signal(SIGINT, sig_handler);

  std::cout << "Starting pipeline..." << std::endl;
  pipeline = new Pipeline();

  std::cout << "Waiting for UDP stream..." << std::endl;
  // Connect to UDP stream
  pipeline->connect();

  std::cout << "Dispatching workers..." << std::endl;
  // Start beamforming thread
  thread worker(naive_seeker, pipeline);

  // Initiate background image
  magnitudeHeatmap.setTo(cv::Scalar(0));

  #if AUDIO 
  init_audio_playback(pipeline);
  #endif

#if CAMERA
  cv::VideoCapture cap(CAMERA_PATH); // Open the default camera (change the
                                     // index if you have multiple cameras)

  if (!cap.isOpened()) {
    std::cerr << "Error: Unable to open the camera." << std::endl;
    return -1;
  }

  while (pipeline->isRunning()) {
    cv::Mat frame;
    cap >> frame; // Capture a frame from the camera

    if (frame.empty()) {
      std::cerr << "Error: Captured frame is empty." << std::endl;
      break;
    }

    Mat overlayImage;
    applyColorMap(magnitudeHeatmap, overlayImage, COLORMAP_JET);

    // Resize the overlay image to match the dimensions of the webcam frame
    cv::resize(overlayImage, overlayImage, frame.size());

    // Overlay the image onto the webcam frame at a specified location (adjust
    // as needed)
    cv::Rect roi(0, 0, overlayImage.cols, overlayImage.rows);
    cv::Mat imageROI = frame(roi);
    cv::addWeighted(imageROI, 1.0, overlayImage, 0.5, 0, imageROI);

    // Display the resulting frame with the overlay
    cv::imshow(APPLICATION_NAME, frame);

    if (waitKey(1) == 'q') {
      // ok = false;
      std::cout << "Stopping" << endl;

      break;
    }
  }

  // Release the camera and close all OpenCV windows
  cap.release();
#else


  // Create a window to display the beamforming data
  cv::namedWindow(APPLICATION_NAME, cv::WINDOW_NORMAL);
  cv::resizeWindow(APPLICATION_NAME, APPLICATION_WIDTH, APPLICATION_HEIGHT);
  int res = 16;

  // Decay image onto previous frame
  cv::Mat previous(Y_RES, X_RES, CV_8UC1);
  previous.setTo(cv::Scalar(0)); // Set to zero
  cv::applyColorMap(previous, previous, cv::COLORMAP_JET);

#if RESIZE_HEATMAP
  cv::resize(previous, previous, cv::Size(), res, res, cv::INTER_LINEAR);
#endif

  std::cout << "Running..." << std::endl;
  while (pipeline->isRunning()) {
    if (canPlot) {
      canPlot = 0;
      cv::Mat coloredMatrix;
      //Blur the image with a Gaussian kernel
      cv::GaussianBlur(magnitudeHeatmap, magnitudeHeatmap, cv::Size(BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE), 0);

      // Apply color map
      cv::applyColorMap(magnitudeHeatmap, coloredMatrix, cv::COLORMAP_JET);

#if RESIZE_HEATMAP
      // Resize to smoothen
      cv::resize(coloredMatrix, coloredMatrix, cv::Size(), res, res, cv::INTER_LINEAR);
#endif
      // Combine previous images for more smooth image
      cv::addWeighted(coloredMatrix, 0.1, previous, 0.9, 0, coloredMatrix);

      // Update previous image
      previous = coloredMatrix;

      // Output image to screen
      cv::imshow(APPLICATION_NAME, coloredMatrix);
    }

    // Check for key press; if 'q' is pressed, break the loop
    if (cv::waitKey(1) == 'q') {
      std::cout << "Stopping application..." << std::endl;
      break;
    }
  }



#endif

#if DEBUG_BEAMFORMER
  // Save all data
  pipeline->save_pipeline("pipeline.bin");
#endif

  std::cout << "Disconnecting pipeline..." << std::endl;
  // Stop UDP stream
  pipeline->disconnect();

  #if AUDIO
  stop_audio_playback();
  #endif

  std::cout << "Closing application..." << std::endl;
  // Close application windows
  cv::destroyAllWindows();

  std::cout << "Waiting for workers..." << std::endl;
  // Join the workers
  worker.join();

  std::cout << "Exiting..." << std::endl;

  // Cleanup
  delete pipeline;
}
