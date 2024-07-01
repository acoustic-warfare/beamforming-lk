#include "config.h"

#include "antenna.h"
#include "delay.h"
#include "options.h"
#include "pipeline.h"

#include "algorithms/mimo.h"

//#include "algorithms/pso_seeker.h"

#if AUDIO
#include "RtAudio.h"
#endif

#include <Eigen/Dense>

#if USE_WARAPS

#include <wara_ps_client.h>

#endif

#include <cmath>
#include <cstdlib>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <signal.h>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>


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

void sig_handler(int sig) {
  // Set the stop_processing flag to terminate worker threads gracefully
  pipeline->disconnect();
}

int main() {
  MatrixXf dome = generate_unit_dome(100);
  MatrixXi lookup_table(90, 360);
  generate_lookup_table(dome, lookup_table);

  std::vector<std::unique_ptr<BeamformingOptions>> options;

#if USE_WARAPS

  WaraPSClient client = WaraPSClient(WARAPS_NAME, WARAPS_ADDRESS);

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

  pipeline->magnitudeHeatmap = &magnitudeHeatmap;  // TODO: in constructor?

  for (int i = 0; i < N_FPGAS; i++) {
    std::cout << "Dispatching workers..." << std::endl;
#if USE_MIMO
    // thread worker(&static_mimo_heatmap_worker, pipeline);
    workers.emplace_back(static_mimo_heatmap_worker, pipeline, i,
                         options[i]->n_sensors_);
#else
   //thread worker(&pso_finder, pipeline, i);
#endif
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

  // Decay image onto previous frame
  cv::Mat previous(Y_RES, X_RES, CV_8UC1);
  previous.setTo(cv::Scalar(0));  // Set to zero
  cv::applyColorMap(previous, previous, cv::COLORMAP_JET);

  cv::Mat frame(Y_RES, X_RES, CV_8UC1);

#if RESIZE_HEATMAP
  cv::resize(frame, frame, cv::Size(), RESOLUTION_MULTIPLIER,
             RESOLUTION_MULTIPLIER, cv::INTER_LINEAR);
  cv::resize(previous, previous, cv::Size(), RESOLUTION_MULTIPLIER,
             RESOLUTION_MULTIPLIER, cv::INTER_LINEAR);
#endif

  std::cout << "Running..." << std::endl;

  while (pipeline->isRunning()) {
    if (pipeline->canPlot) {
      pipeline->canPlot = 0;
      cv::Mat smallFrame;

      // Blur the image with a Gaussian kernel
      cv::GaussianBlur(magnitudeHeatmap, magnitudeHeatmap,
                       cv::Size(BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE), 0);

      // Apply color map
      cv::applyColorMap(magnitudeHeatmap, smallFrame, cv::COLORMAP_JET);

#if RESIZE_HEATMAP
      // Resize to smoothen
      cv::resize(smallFrame, frame, cv::Size(), RESOLUTION_MULTIPLIER,
                 RESOLUTION_MULTIPLIER, cv::INTER_LINEAR);
#endif
      // Combine previous images for more smooth image
      cv::addWeighted(frame, IMAGE_CURRENT_WEIGHTED_RATIO, previous,
                      IMAGE_PREVIOUS_WEIGHTED_RATIO, 0, frame);

      // Update previous image
      previous = frame;
    }

#if CAMERA
    cap >> cameraFrame;  // Capture a frame from the camera

    if (cameraFrame.empty()) {
      std::cerr << "Error: Captured frame is empty." << std::endl;
      break;
    }

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
  }

#if USE_WARAPS
  client_thread.join();
#endif

  std::cout << "Exiting..." << std::endl;

  // Cleanup
  delete pipeline;
}
