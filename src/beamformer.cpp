

#include "antenna.h"
#include "config.h"
#include "delay.h"
#include "mimo.h"
#include "options.h"
#include "pipeline.h"

#if AUDIO
#include "RtAudio.h"
#endif

#include <Eigen/Dense>

#if USE_WARAPS
#include <wara_ps_client.h>
#endif

#include <signal.h>

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#define VALID_SENSOR(i) (64 <= i) && (i < 128)

/**
Main application for running beamformer program

You may try to use:

sudo chrt -f 98 ./beamformer

for better performace by altering the real-time scheduling attributes of the
program.

 */

std::atomic_int canPlot = 0;

Pipeline *pipeline;
std::vector<std::thread> workers;

// Intermediate heatmap used for beamforming (8-bit)
cv::Mat magnitudeHeatmap(Y_RES, X_RES, CV_8UC1);

/**
 * @brief Convert multiple input streams into single level by delay
 *
 * @param t_id [TODO:parameter]
 * @param task pool partition
 * @param fractional_delays delays to use
 * @param rb ring buffer to use
 * @return power level
 */
float miso(int t_id, int task, int *offset_delays, float *fractional_delays,
           Streams *streams, uint32_t n_sensors) {
  float out[N_SAMPLES] = {0.0};
  int n = 0;
  for (int s = 0; s < n_sensors; s++) {
    // if (!((s == 64) || (s == 64 + 8) || (s == 127 - 16) || (s == 127))) {
    //   continue;
    // }

    if (VALID_SENSOR(s)) {
      float fraction = fractional_delays[s - 64];
      int offset = offset_delays[s - 64];

      float *signal = (float *)((char *)streams->buffers[s] +
                                streams->position + offset * sizeof(float));

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

/**
 * Beamforming as fast as possible on top of pipeline
 */
void static_mimo_heatmap_worker(Pipeline *pipeline, int stream_id,
                                uint32_t n_sensors) {
  Antenna antenna = create_antenna(Position(0, 0, 0), COLUMNS, ROWS, DISTANCE);

  std::cout << "mimo: " << n_sensors << std::endl;

  float fractional_delays[X_RES * Y_RES * n_sensors];
  int offset_delays[X_RES * Y_RES * n_sensors];

  compute_scanning_window(&offset_delays[0], &fractional_delays[0], antenna,
                          FOV, X_RES, Y_RES, n_sensors);

  int max = X_RES * Y_RES;

  float image[X_RES * Y_RES];

  int pixel_index = 0;

  int newData;
  float power;
  float threshold = 3e-8;

  float norm = 1 / 1e-05;

  float maxVal = 1.0;

  Streams *streams = pipeline->getStreams(stream_id);
  std::cout << "streams: " << streams << std::endl;

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
      int task = pixel_index * n_sensors;

      xi = pixel_index % X_RES;
      yi = pixel_index / X_RES;

      // Get power level from direction
      float val = miso(0, pixel_index, &offset_delays[task],
                       &fractional_delays[task], streams, n_sensors);

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
      magnitudeHeatmap.at<uchar>(yi, xi) = (uchar)(power * 255);

      pixel_index++;
      pixel_index %= X_RES * Y_RES;

      i++;
    }

    canPlot = 1;

    norm = (1 - alpha) * norm + alpha * (1 / (maxVal));

    // norm = (1/maxVal) * 1.1f;

    // cout << maxVal << endl;
  }
}

void sig_handler(int sig) {
  // Set the stop_processing flag to terminate worker threads gracefully
  pipeline->disconnect();
}

int main() {
  // BeamformingOptions options[N_FPGAS];
  // std::vector<BeamformingOptions> options;
  // for (int i = 0; i < N_FPGAS; i++) {
  //   options.emplace_back();
  // }
  //  BeamformingOptions options;

  std::vector<std::unique_ptr<BeamformingOptions>> options;

#if USE_WARAPS

  WaraPSClient client = WaraPSClient("test", "mqtt://test.mosquitto.org:1883");

  client.SetCommandCallback("focus_bf", [&](const nlohmann::json &payload) {
    float theta = payload["theta"];
    float phi = payload["phi"];
    float duration =
        payload.contains("duration") ? (float)payload["duration"] : 5.0f;

    cout << "Theta: " << theta << "\nPhi: " << phi << endl;
    client.PublishMessage("exec/response", string("Focusing beamformer for " +
                                                  to_string(duration)));
  });

  thread client_thread = client.Start();
#endif

  // Setup sigint i.e Ctrl-C
  signal(SIGINT, sig_handler);

  std::cout << "Starting pipeline..." << std::endl;
  pipeline = new Pipeline();

  std::cout << "Waiting for UDP stream..." << std::endl;
  // Connect to UDP stream
  pipeline->connect(options);
  std::cout << "\rn_sensors_ 0 MAIN: " << options[0]->n_sensors_ << std::endl;
  std::cout << "\rn_sensors_ 1 MAIN: " << options[1]->n_sensors_ << std::endl;
  // std::cout << "\rn_sensors_ MAIN: " << options[3]->n_sensors_ << std::endl;

  for (int i = 0; i < N_FPGAS; i++) {
    std::cout << "Dispatching workers..." << std::endl;
    // Start beamforming thread
    // thread worker(static_mimo_heatmap_worker, pipeline, i);
    workers.emplace_back(static_mimo_heatmap_worker, pipeline, i,
                         options[i]->n_sensors_);
  }

  // Initiate background image
  magnitudeHeatmap.setTo(cv::Scalar(0));

#if AUDIO
  init_audio_playback(pipeline);
#endif

#if CAMERA
  cv::VideoCapture cap(CAMERA_PATH);  // Open the default camera (change the
                                      // index if you have multiple cameras)
  if (!cap.isOpened()) {
    std::cerr << "Error: Unable to open the camera." << std::endl;
    // goto cleanup;
    exit(1);
  }

  cv::Mat cameraFrame;
#endif

  // Create a window to display the beamforming data
  cv::namedWindow(APPLICATION_NAME, cv::WINDOW_NORMAL);
  cv::resizeWindow(APPLICATION_NAME, APPLICATION_WIDTH, APPLICATION_HEIGHT);
  int res = 16;

  // Decay image onto previous frame
  cv::Mat previous(Y_RES, X_RES, CV_8UC1);
  previous.setTo(cv::Scalar(0));  // Set to zero
  cv::applyColorMap(previous, previous, cv::COLORMAP_JET);

  cv::Mat frame(Y_RES, X_RES, CV_8UC1);

#if RESIZE_HEATMAP
  cv::resize(frame, frame, cv::Size(), res, res, cv::INTER_LINEAR);
  cv::resize(previous, previous, cv::Size(), res, res, cv::INTER_LINEAR);
#endif

  std::cout << "Running..." << std::endl;

  while (pipeline->isRunning()) {
    if (canPlot) {
      canPlot = 0;
      cv::Mat smallFrame;

      // Blur the image with a Gaussian kernel
      cv::GaussianBlur(magnitudeHeatmap, magnitudeHeatmap,
                       cv::Size(BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE), 0);

      // Apply color map
      cv::applyColorMap(magnitudeHeatmap, smallFrame, cv::COLORMAP_JET);

#if RESIZE_HEATMAP
      // Resize to smoothen
      cv::resize(smallFrame, frame, cv::Size(), res, res, cv::INTER_LINEAR);
#endif
      // Combine previous images for more smooth image
      cv::addWeighted(frame, 0.1, previous, 0.9, 0, frame);

      // Update previous image
      previous = frame;
    }

#if CAMERA
    cap >> cameraFrame;  // Capture a frame from the camera

    if (cameraFrame.empty()) {
      std::cerr << "Error: Captured frame is empty." << std::endl;
      break;
    }

    // Overlay the image onto the webcam frame at a specified location (adjust
    // as needed)
    // cv::Rect roi(0, 0, frame.cols, frame.rows);
    // cv::Mat imageROI = cameraFrame(roi);
    // cv::addWeighted(imageROI, 1.0, frame, 0.5, 0, imageROI);

    cv::resize(cameraFrame, cameraFrame, cv::Size(frame.cols, frame.rows), 0, 0,
               cv::INTER_LINEAR);
    cv::addWeighted(cameraFrame, 1.0, frame, 0.5, 0, frame);
#endif

    // Output image to screen
    cv::imshow(APPLICATION_NAME, frame);

    // Check for key press; if 'q' is pressed, break the loop
#if USE_WARAPS
    if (!client.running() || cv::waitKey(1) == 'q') {
#else
    if (cv::waitKey(1) == 'q') {
#endif
      std::cout << "Stopping application..." << std::endl;
      break;
    }
  }

#if DEBUG_BEAMFORMER
  // Save all data
  pipeline->save_pipeline("pipeline.bin");
#endif

#if AUDIO
  stop_audio_playback();
#endif

  std::cout << "Closing application..." << std::endl;
  // Close application windows
  cv::destroyAllWindows();

cleanup:

  std::cout << "Disconnecting pipeline..." << std::endl;
  // Stop UDP stream
  pipeline->disconnect();

  std::cout << "Waiting for workers..." << std::endl;
  // Join the workers
  // worker.join();

  for (auto &worker : workers) {
    if (worker.joinable()) {
      worker.join();
    }

#if USE_WARAPS
    client_thread.join();
#endif

    std::cout << "Exiting..." << std::endl;

    // Cleanup
    delete pipeline;
  }
}
