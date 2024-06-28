

#include "config.h"

#include "antenna.h"
#include "delay.h"
#include "options.h"
#include "pipeline.h"

#include "mimo.h"
#include "kf.h"

#include "algorithms/pso_seeker.h"

#if AUDIO
#include "RtAudio.h"
#endif

#include <Eigen/Dense>

#if USE_WARAPS
#include <wara_ps_client.h>
#endif 

#include <psocpp.h>

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
           Streams *streams) {
  float out[N_SAMPLES] = {0.0};
  int n = 0;
  for (int s = 0; s < N_SENSORS; s++) {

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









inline float clip(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}


void pso_finder(Pipeline *pipeline) {
  Antenna antenna = create_antenna(Position(0, 0, 0), COLUMNS, ROWS, DISTANCE);
  Streams *streams = pipeline->getStreams();

  PSO pso(40, antenna, streams);

  int newData;

  int prevX = 0;
  int prevY = 0;

  while (pipeline->isRunning()) {

    // Wait for incoming data
    pipeline->barrier();

    // This loop may run until new data has been produced, meaning its up to
    // the machine to run as fast as possible
    newData = pipeline->mostRecent();

    pso.initialize_particles();

    pso.optimize(30);

    Eigen::Vector3f sample = pso.sanitize();

    float theta = sample(0);
    float phi = sample(1);

    theta = clip(theta, -ANGLE_LIMIT, ANGLE_LIMIT);
    phi = clip(phi, -ANGLE_LIMIT, ANGLE_LIMIT);

    //float x = (float)(cos((double)theta) * sin((double)phi));
    //float y = (float)(sin((double)theta) * sin((double)phi));

    //float x = (float)(cos((double)pso.global_best_theta) * sin((double)pso.global_best_phi));
    //float y = (float)(sin((double)pso.global_best_theta) * sin((double)pso.global_best_phi));
    
    //int xi = (int)((x + 1.0) / 2.0 * X_RES);
    //int yi = (int)((y + 1.0) / 2.0 * Y_RES);

    magnitudeHeatmap.setTo(cv::Scalar(0));

    for (auto& particle : pso.particles) {
      theta = particle.best_theta;
      phi = particle.best_phi;
      int xi = (int)((double)X_RES * ((theta) + to_radians(FOV / 2)) / 2.0);
      int yi = (int)((double)Y_RES * ((phi) + to_radians(FOV / 2)) / 2.0);
      
      //magnitudeHeatmap.at<uchar>(prevY, prevX) = (uchar)(0);
      //magnitudeHeatmap.at<uchar>(yi, xi) = (uchar)(255);
      magnitudeHeatmap.at<uchar>(Y_RES - 1 - yi, xi) = (uchar)(255);
    }

    //int xi = (int)((double)X_RES * ((theta) + to_radians(FOV / 2)) / 2.0);
    //int yi = (int)((double)Y_RES * ((phi) + to_radians(FOV / 2)) / 2.0);
    //
    ////magnitudeHeatmap.at<uchar>(prevY, prevX) = (uchar)(0);
    ////magnitudeHeatmap.at<uchar>(yi, xi) = (uchar)(255);
    //magnitudeHeatmap.at<uchar>(Y_RES - 1 - yi, xi) = (uchar)(255);
    //prevX = xi;
    //prevY = yi;
    canPlot = 1;
    //std::cout << "(" << x << ", " << y << ")" << std::endl;
    std::cout << "Theta: " << pso.global_best_theta << " Phi: " << pso.global_best_phi << std::endl;
  }

}

/**
 * Beamforming as fast as possible on top of pipeline
 */
void static_mimo_heatmap_worker(Pipeline *pipeline) {

  Antenna antenna = create_antenna(Position(0, 0, 0), COLUMNS, ROWS, DISTANCE);

  float fractional_delays[X_RES * Y_RES * N_SENSORS];
  int offset_delays[X_RES * Y_RES * N_SENSORS];

  compute_scanning_window(&offset_delays[0], &fractional_delays[0], antenna,
                          FOV, X_RES, Y_RES);

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
      float val = miso(0, pixel_index, &offset_delays[task],
                       &fractional_delays[task], streams);

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
  
  BeamformingOptions options;

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
  pipeline->connect();

  std::cout << "Dispatching workers..." << std::endl;

#if USE_MIMO
  // Start beamforming thread
  thread worker(static_mimo_heatmap_worker, pipeline);
#else
  thread worker(pso_finder, pipeline);
#endif
  

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
    //goto cleanup;
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
  previous.setTo(cv::Scalar(0)); // Set to zero
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
      cv::resize(smallFrame, frame, cv::Size(), res, res,
                 cv::INTER_LINEAR);
#endif
      // Combine previous images for more smooth image
      cv::addWeighted(frame, 0.1, previous, 0.9, 0, frame);

      // Update previous image
      previous = frame;

      
    }

#if CAMERA 
    cap >> cameraFrame; // Capture a frame from the camera

    if (cameraFrame.empty()) {
      std::cerr << "Error: Captured frame is empty." << std::endl;
      break;
    }

    // Overlay the image onto the webcam frame at a specified location (adjust
    // as needed)
    //cv::Rect roi(0, 0, frame.cols, frame.rows);
    //cv::Mat imageROI = cameraFrame(roi);
    //cv::addWeighted(imageROI, 1.0, frame, 0.5, 0, imageROI);

    cv::resize(cameraFrame, cameraFrame, cv::Size(frame.cols,frame.rows), 0, 0, cv::INTER_LINEAR);
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
  worker.join();

#if USE_WARAPS
  client_thread.join();
#endif

  std::cout << "Exiting..." << std::endl;

  // Cleanup
  delete pipeline;
}
