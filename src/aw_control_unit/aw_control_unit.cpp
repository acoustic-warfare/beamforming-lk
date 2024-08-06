/** @file aw_control_unit.cpp
 * @author Janne, Irreq, Tuva
 * @date 2024-07-04
*/

#include "aw_control_unit.h"

#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <stdexcept>

#include "target_handler.h"

#define USE_AUDIO true

#define APPLICATION_NAME "Beamforming"
#define APPLICATION_WIDTH 1024
#define APPLICATION_HEIGHT 1024

#define X_RES 1024
#define Y_RES 1024

#define BLUR_KERNEL_SIZE 5//Kernel size of blur on heatmap
#define BLUR_EFFECT false

/**
 * Create a name for the video file
 */
std::string generateUniqueFilename() {
    // Get the current time
    std::time_t now = std::time(nullptr);
    std::tm* localTime = std::localtime(&now);

    // Format the time into a string
    std::ostringstream oss;
    oss << std::put_time(localTime, "%Y%m%d_%H%M%S");// Format: YYYYMMDD_HHMMSS

    // Create the filename with the .avi extension
    std::string filename = oss.str() + ".avi";

    return filename;
}

/**
 * Initiate the videostream writer
 */
int startRecording(cv::VideoWriter& videoWriter, cv::Size frame_size, double fps) {

    // Define the codec and create VideoWriter object to write the video to a file
    int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');// Codec type                                   // Frames per second
    videoWriter = cv::VideoWriter(generateUniqueFilename(), codec, fps, frame_size);

    if (!videoWriter.isOpened()) {
        std::cerr << "Error: Could not open the video file for write." << std::endl;
        return -1;
    }

    return 0;
}


/**
 * Stop the videostream writer
 */
void stopRecording(cv::VideoWriter& videoWriter) {
    videoWriter.release();
}

//TODO: Add logo
void AWControlUnit::Start(const std::vector<int>& ports, const std::string& ip_address, bool use_camera, const std::string& camera,
                          bool audio, bool mimo, bool tracking, int mimo_res, bool verbose, bool record, float fov) {
    cv::VideoCapture cap;
    cv::VideoWriter videoWriter;

    bool running = true;
    bool recording = false;

    int delay = 1;
    double fps = 60.0;


    if (use_camera) {
        fov = FOV;
        cap = cv::VideoCapture(camera);
        if (!cap.isOpened()) {
            std::cerr << "Error: Could not open the webcam." << std::endl;
        }

        // Get the frames per second of the video
        fps = cap.get(cv::CAP_PROP_FPS);

        // Calculate the delay between frames
        delay = 1000 / fps;

        // Get the width and height of frames from the webcam
        int frame_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
        int frame_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
        cv::Size frame_size(frame_width, frame_height);
    }


    std::vector<AWProcessingUnit*> awpus;

    int i = 0;
    for (const int port: ports) {
        const char* addr = ip_address.c_str();
        if (verbose) {
            std::cout << "Starting MIMO: " << port << std::endl;
        }

        AWProcessingUnit* awpu = new AWProcessingUnit(ip_address.c_str(), port, fov, mimo_res, verbose, audio);
        awpus.push_back(awpu);

        if (tracking) { awpu->start(GRADIENT); }
        if (mimo) { awpu->start(MIMO); }
        //if (audio) awpu.play_audio();
    }

    int awpu_count = awpus.size();

    //if constexpr (USE_AUDIO) {
    //    awpu1.play_audio();
    //}

    namedWindow(APPLICATION_NAME, cv::WINDOW_NORMAL);

    cv::Mat frame(Y_RES, X_RES, CV_8UC1);
    cv::Mat colorFrame(Y_RES, X_RES, CV_8UC1);//TODO: should not be same as line 62-64

    cv::Mat combinedFrame;
    //cv::Mat colorFrame;
    if (awpu_count != 0) {
        combinedFrame = cv::Mat(Y_RES, X_RES * awpu_count, CV_8UC1);
        colorFrame = cv::Mat(Y_RES, X_RES * awpu_count, CV_8UC1);
        cv::resizeWindow(APPLICATION_NAME, APPLICATION_WIDTH * awpu_count, APPLICATION_HEIGHT);

    } else {
        combinedFrame = cv::Mat(Y_RES, X_RES, CV_8UC1);
        colorFrame = cv::Mat(Y_RES, X_RES, CV_8UC1);
        cv::resizeWindow(APPLICATION_NAME, APPLICATION_WIDTH, APPLICATION_HEIGHT);
    }

    std::vector<cv::Mat> smallFrames;
    std::vector<cv::Mat> bigFrames;

    colorFrame.setTo(cv::Scalar(0));
    combinedFrame.setTo(cv::Scalar(0));

    for (int i = 0; i < awpu_count; i++) {
        smallFrames.push_back(cv::Mat(mimo_res, mimo_res, CV_8UC1));
        bigFrames.push_back(cv::Mat(Y_RES, X_RES, CV_8UC1));
    }

    auto start = std::chrono::high_resolution_clock::now();
    int frameCount = 0;

    if (awpu_count > 1) {
        targetHandler_.AddAWPU(awpus[0], {1, 0, 0})//TODO: hardcoded
                .AddAWPU(awpus[1], {-1, 0, 0})     //TODO: hardcoded
                .Start();
    }

    if (usingWaraPS_) {
        data_thread_ = std::thread([this] {
            while (client_.running()) {
                publishData();
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        });
        targetHandler_.DisplayToWaraPS(true);
    }

    while (running && ((usingWaraPS_ && client_.running()) || !usingWaraPS_)) {
        cv::Mat frame;

        if (use_camera) {
            cap >> frame;// Capture a new frame from the webcam

            if (frame.empty()) {
                std::cerr << "Error: Could not read a frame from the webcam." << std::endl;
                break;
            }

            if (recording && awpu_count == 0) {
                videoWriter.write(frame);// Write the frame to the video file
            }
        }


        for (int i = 0; i < awpu_count; i++) {

            // Reset frames
            smallFrames[i].setTo(cv::Scalar(0));
            bigFrames[i].setTo(cv::Scalar(0));

            // Draw onto frames
            awpus[i]->draw(&smallFrames[i], &bigFrames[i]);

#if BLUR_EFFECT
            // Blur the image with a Gaussian kernel
            cv::GaussianBlur(bigFrames[i], bigFrames[i],
                             cv::Size(BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE), 0);
#endif
            // Pretty colors
            cv::applyColorMap(bigFrames[i], bigFrames[i], cv::COLORMAP_OCEAN);

            // Overlay onto camera
            if (use_camera) {
                cv::resize(frame, frame, cv::Size(bigFrames[i].cols, bigFrames[i].rows), 0, 0, cv::INTER_LINEAR);
                cv::addWeighted(frame, 1.0, bigFrames[i], 0.5, 0, bigFrames[i]);
            } else {

                // Create a circle around view

                // Create a mask with the same dimensions as the image, initialized to black
                cv::Mat mask = cv::Mat::zeros(bigFrames[i].size(), bigFrames[i].type());

                // Define the circle parameters
                int radius = bigFrames[i].rows / 2;                                  // Radius of the circles
                cv::Point center1(radius, bigFrames[i].rows / 2);                    // Center of the first circle
                cv::Point center2(bigFrames[i].cols - radius, bigFrames[i].rows / 2);// Center of the second circle

                // Draw filled white circles on the mask
                cv::circle(mask, center1, radius, cv::Scalar(255, 255, 255), -1);
                cv::circle(mask, center2, radius, cv::Scalar(255, 255, 255), -1);

                // Apply the mask to the original image
                cv::Mat result;
                bigFrames[i].copyTo(result, mask);
                bigFrames[i] = result;
            }

            // Define font face
            int fontFace = cv::FONT_HERSHEY_SIMPLEX;

            // Define font scale (size)
            double fontScale = 1;

            // Define font color (here it's white)
            cv::Scalar color(255, 255, 255);

            std::stringstream ss;

            ss << "Trackers: " << awpus[i]->targets().size();
            cv::putText(bigFrames[i], ss.str(), cv::Point(0, 20), fontFace, fontScale, color, 2);
        }

        if (awpu_count == 0) {
            combinedFrame = frame;
        } else if (awpu_count == 1) {
            combinedFrame = bigFrames[0];
        } else {
            cv::hconcat(bigFrames[0], bigFrames[1], combinedFrame);
        }


        if (recording) {
            // Write the frame to the video file
            videoWriter.write(combinedFrame);
        }

        // Calculate FPS
        frameCount++;
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        if (elapsed.count() >= 1.0) {
            fps = frameCount / elapsed.count();
            frameCount = 0;
            start = end;
        }

        // Display FPS on the frame
        std::string fpsText = "FPS: " + std::to_string(fps);
        cv::putText(combinedFrame, fpsText, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);


        // Display the frame
        cv::imshow(APPLICATION_NAME, combinedFrame);

        // Wait for a key press and get the ASCII code
        int key = cv::waitKey(delay);

        // Check for specific keys
        switch (key) {
            case 'q':// Quit the application
                std::cout << "Quit key pressed ('q'). Exiting..." << std::endl;
                running = false;
                break;
            case 'r':
                if (recording) {
                    stopRecording(videoWriter);
                    recording = false;
                    std::cout << "Stopped recording" << std::endl;
                } else {
                    if (use_camera && awpu_count == 0) {
                        int frame_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
                        int frame_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
                        cv::Size frame_size(frame_width, frame_height);
                        startRecording(videoWriter, frame_size, fps);
                    } else {
                        startRecording(videoWriter, combinedFrame.size(), fps);
                    }

                    recording = true;
                    std::cout << "Started recording" << std::endl;
                }
                break;
            default:
                // Handle other keys or no key press
                break;
        }
    }

    if (use_camera) {
        cap.release();
    }

    if (recording) {
        stopRecording(videoWriter);
    }

    if (usingGps_) {
        gps_stream(&gpsData_, WATCH_DISABLE, nullptr);
        gps_close(&gpsData_);
    }

    if (usingWaraPS_) {
        client_.Stop();
        targetHandler_.Stop();
        data_thread_.join();
    }

    for (auto& awpu: awpus) {
        delete awpu;
    }

    std::cout << "awpus" << std::endl;

    cv::destroyAllWindows();

    //if constexpr (USE_AUDIO) {
    //    awpu1.stop_audio();
    //}
}

// Publishes data every second
void AWControlUnit::publishData() {
    if (usingGps_ && gps_waiting(&gpsData_, 1000)) {
        if (gps_read(&gpsData_, nullptr, 0) == -1) {
            std::cerr << "GPS Read error" << std::endl;
        } else if (isfinite(gpsData_.fix.altitude) &&
                   isfinite(gpsData_.fix.latitude) &&
                   isfinite(gpsData_.fix.longitude)) {
            // Sending NaN breaks WARA PS Arena

            const nlohmann::json gpsJson = {
                    {"longitude", gpsData_.fix.longitude},
                    {"latitude", gpsData_.fix.latitude},
                    {"altitude", gpsData_.fix.altitude},
                    {"type", "GeoPoint"}};
            client_.PublishMessage("sensor/position", gpsJson.dump(4));
        }
    }

    client_.PublishMessage("sensor/heading", std::to_string(90.0));// Currently not a real value
    client_.PublishMessage("sensor/course", std::to_string(0));
    client_.PublishMessage("sensor/speed", std::to_string(0));
    client_.PublishMessage("sensor/camera_tags", "[ \"LJUDKRIGET\" ]");
}

AWControlUnit::AWControlUnit() : client_(WARAPS_NAME, WARAPS_ADDRESS,
                                         std::getenv("MQTT_USERNAME") == nullptr ? "" : std::getenv("MQTT_USERNAME"),
                                         std::getenv("MQTT_PASSWORD") == nullptr ? "" : std::getenv("MQTT_PASSWORD")),
                                 targetHandler_(&gpsData_) {
    int gpsError = gps_open(GPS_ADDRESS, std::to_string(GPS_PORT).c_str(), &gpsData_);
    gpsError |= gps_stream(&gpsData_, WATCH_ENABLE | WATCH_JSON, nullptr);

    if (gpsError != 0) {
        std::cerr << "GPS Error: " << gps_errstr(gpsError) << std::endl;
        std::cerr << "Continuing without gps" << std::endl;
        usingGps_ = false;
    } else {
        usingGps_ = true;
    }

    try {
        client_.Start();
        usingWaraPS_ = true;
    } catch (std::runtime_error& e) {
        std::cerr << "WARA PS Connection error: " << e.what() << std::endl;
        std::cout << "Continuing without WARA PS Connection" << std::endl;
        usingWaraPS_ = false;
    }
}
