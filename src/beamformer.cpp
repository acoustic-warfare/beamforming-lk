

#include <Eigen/Dense>

#include "algorithms/pso_seeker.h"
#include "antenna.h"
#include "audio/audio_wrapper.h"
#include "config.h"
#include "delay.h"
#include "mimo.h"
#include "options.h"
#include "pipeline.h"

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

// Intermediate heatmap used for beamforming (8-bit)
cv::Mat magnitudeHeatmap(Y_RES, X_RES, CV_8UC1);


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

        float azimuth = sample(0);
        float elevation = sample(1);

        azimuth = clip(azimuth, -ANGLE_LIMIT, ANGLE_LIMIT);
        elevation = clip(elevation, -ANGLE_LIMIT, ANGLE_LIMIT);

        //float x = (float)(cos((double)theta) * sin((double)phi));
        //float y = (float)(sin((double)theta) * sin((double)phi));

        //float x = (float)(cos((double)pso.global_best_theta) * sin((double)pso.global_best_phi));
        //float y = (float)(sin((double)pso.global_best_theta) * sin((double)pso.global_best_phi));

        //int xi = (int)((x + 1.0) / 2.0 * X_RES);
        //int yi = (int)((y + 1.0) / 2.0 * Y_RES);


        magnitudeHeatmap.setTo(cv::Scalar(0));

        for (auto &particle: pso.particles) {
            azimuth = particle.best_azimuth;
            elevation = particle.best_elevation;
            int xi = (int) ((double) X_RES * (azimuth + ANGLE_LIMIT) / 2.0);
            int yi = (int) ((double) Y_RES * (elevation + ANGLE_LIMIT) / 2.0);

            //magnitudeHeatmap.at<uchar>(prevY, prevX) = (uchar)(0);
            //magnitudeHeatmap.at<uchar>(yi, xi) = (uchar)(255);
            magnitudeHeatmap.at<uchar>(Y_RES - 1 - yi, xi) = (uchar) (255);
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
    }
}


void sig_handler(int sig) {
    // Set the stop_processing flag to terminate worker threads gracefully
    pipeline->disconnect();
}

int main() {
    MatrixXf dome = generate_unit_dome(100);
    // MatrixXi lookup_table(90, 360);
    // generate_lookup_table(dome, lookup_table);

    BeamformingOptions options;

#if USE_WARAPS

    WaraPSClient client = WaraPSClient("test", "mqtt://test.mosquitto.org:1883");

    client.SetCommandCallback("focus_bf", [&](const nlohmann::json &payload) {
        float theta = payload["theta"];
        float phi = payload["phi"];
        float duration =
                payload.contains("duration") ? (float) payload["duration"] : 5.0f;

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
#if USE_MIMO  // Start beamforming thread
    thread worker(static_mimo_heatmap_worker, pipeline, std::ref(magnitudeHeatmap), canPlot);
#else
    thread worker(pso_finder, pipeline);
#endif


    // Initiate background image
    magnitudeHeatmap.setTo(cv::Scalar(0));

    AudioWrapper audio(*pipeline->getStreams());
    if(options.audio_on_) {
        audio.start_audio_playback();
    }


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
            cv::addWeighted(frame, IMAGE_CURRENT_WEIGHTED_RATIO, previous, IMAGE_PREVIOUS_WEIGHTED_RATIO, 0, frame);

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

    std::cout << "Closing application..." << std::endl;
    // Close application windows
    cv::destroyAllWindows();

    if(options.audio_on_) {
        audio.stop_audio_playback();
    }

    cleanup:

    std::cout << "Disconnecting pipeline..." << std::endl;
    // Stop UDP stream
    pipeline->disconnect();

    std::cout << "Waiting for workers..." << std::endl;
    // Unite the proletariat
    worker.join();

#if USE_WARAPS
    client_thread.join();
#endif

    std::cout << "Exiting..." << std::endl;

    // Cleanup
    delete pipeline;
}
