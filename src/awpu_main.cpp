#include <ctime>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>

#include "argparse/argparse.hpp"
#include "awpu.h"
#include "config.h"


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

void stopRecording(cv::VideoWriter& videoWriter) {
    videoWriter.release();
}

int main(int argc, char* argv[]) {
    argparse::ArgumentParser program(argv[0]);

    program.add_argument("--camera")
            .default_value(std::string("false"))
            .help("Camera option, default is false");

    program.add_argument("--ip-address")
            .default_value(std::string("127.0.0.1"))
            .help("IP address");

    program.add_argument("--audio")
            .default_value(false)
            .implicit_value(true)
            .help("Audio option");

    program.add_argument("--mimo")
            .default_value(false)
            .implicit_value(true)
            .help("MIMO option");

    program.add_argument("--record")
            .default_value(false)
            .implicit_value(true)
            .help("Record option");

    program.add_argument("--mimo-res")
            .default_value(100)
            .scan<'i', int>()
            .help("MIMO Resolution");

    program.add_argument("--tracking")
            .default_value(false)
            .implicit_value(true)
            .help("Tracking option");

    program.add_argument("--fov")
            .scan<'f', float>()
            .default_value(90.0f)
            .help("Field of view");

    program.add_argument("--port")
            .append()
            .scan<'i', int>()
            .help("PORT numbers");

    try {
        program.parse_args(argc, argv);
    } catch (const std::exception& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        std::exit(1);
    }

    // Retrieve the parsed arguments
    std::string camera = program.get<std::string>("--camera");
    std::string ip_address = program.get<std::string>("--ip-address");
    bool audio = program.get<bool>("--audio");
    bool mimo = program.get<bool>("--mimo");
    bool tracking = program.get<bool>("--tracking");
    float fov = program.get<float>("--fov");
    int mimo_res = program.get<int>("--mimo-res");
    bool record = program.get<bool>("--record");
    std::vector<int> ports = program.get<std::vector<int>>("--port");


    // Display the parsed argument values
    std::cout << "Camera: " << camera << std::endl;
    std::cout << "IP Address: " << ip_address << std::endl;
    std::cout << "Audio: " << audio << std::endl;
    std::cout << "MIMO: " << mimo << std::endl;
    std::cout << "Tracking: " << tracking << std::endl;
    std::cout << "FOV: " << fov << std::endl;
    std::cout << "Ports: ";
    for (const auto& port: ports) {
        std::cout << port << " ";
    }
    std::cout << std::endl;

    bool use_camera = (camera.compare("false") != 0);

    std::cout << "Use camera: " << use_camera << std::endl;

    cv::VideoCapture cap;
    cv::VideoWriter videoWriter;

    bool running = true;
    bool recording = false;

    int delay = 1;
    double fps = 60.0;


    if (use_camera) {
        // Open the default webcam
        cap = cv::VideoCapture(camera);
        if (!cap.isOpened()) {
            std::cerr << "Error: Could not open the webcam." << std::endl;
            return -1;
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



    std::vector<AWProcessingUnit> awpus;

    int i = 0;
    for (const int port : ports) {
        const char *addr = ip_address.c_str();
        std::cout << "Starting MIMO: " << port << std::endl;
        awpus.emplace_back(ip_address.c_str(), port, fov, mimo_res);
        AWProcessingUnit &awpu = awpus.back();
        if (tracking) { awpu.start(GRADIENT); }
        if (mimo) { awpu.start(MIMO); }
        //AWProcessingUnit* awpu = new AWProcessingUnit(ip_address.c_str(), port, fov);
        //if (tracking) { awpu->start(GRADIENT); }
        //if (mimo) { awpu->start(MIMO); }
        //awpus.push_back(awpu);
    }

    int awpu_count = awpus.size();

    // Create a window to display the beamforming data
    cv::namedWindow(APPLICATION_NAME, cv::WINDOW_NORMAL);

    cv::Mat combinedFrame;
    cv::Mat colorFrame;
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

    while (running) {

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
            smallFrames[i].setTo(cv::Scalar(0));
            //cv::resize(bigFrames[i], bigFrames[i], bigFrames[i].size(), 0, 0, cv::INTER_LINEAR);
            bigFrames[i].setTo(cv::Scalar(0));
            //awpus[i]->draw_heatmap(&smallFrames[i]);
            //awpus[i]->draw(&smallFrames[i], &bigFrames[i]);
            awpus[i].draw(&smallFrames[i], &bigFrames[i]);
            //cv::Mat tmp;
            //cv::resize(smallFrames[i], tmp, bigFrames[i].size(), 0, 0, cv::INTER_LINEAR);
            //cv::Mat result;
            //bigFrames[i].copyTo(result, tmp);
            //tmp.copyTo(bigFrames[i], tmp);
            //bigFrames[i] = result;
            //bigFrames[i] = tmp;
            //cv::resize(smallFrames[i], bigFrames[i], bigFrames[i].size(), 0, 0, cv::INTER_LINEAR);
            //cv::addWeighted(tmp, 1.0, bigFrames[i], 1.0, 0, bigFrames[i]);
            // Blur the image with a Gaussian kernel
            cv::GaussianBlur(bigFrames[i], bigFrames[i],
                             cv::Size(BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE), 0);
            cv::applyColorMap(bigFrames[i], bigFrames[i], cv::COLORMAP_JET);

            if (use_camera) {
                cv::resize(frame, frame, cv::Size(bigFrames[i].cols, bigFrames[i].rows), 0, 0, cv::INTER_LINEAR);
                cv::addWeighted(frame, 1.0, bigFrames[i], 0.5, 0, bigFrames[i]);
            } else {

                // Create a mask with the same dimensions as the image, initialized to black
                cv::Mat mask = cv::Mat::zeros(bigFrames[i].size(), bigFrames[i].type());

                // Define the circle parameters
                int radius = bigFrames[i].rows / 2;                                // Radius of the circles
                cv::Point center1(radius, bigFrames[i].rows / 2);                  // Center of the first circle
                cv::Point center2(bigFrames[i].cols - radius, bigFrames[i].rows / 2);// Center of the second circle

                // Draw filled white circles on the mask
                cv::circle(mask, center1, radius, cv::Scalar(255, 255, 255), -1);
                cv::circle(mask, center2, radius, cv::Scalar(255, 255, 255), -1);

                // Apply the mask to the original image
                cv::Mat result;
                bigFrames[i].copyTo(result, mask);
                bigFrames[i] = result;
            }
            
        }

        if (awpu_count == 0) {
            combinedFrame = frame;
        } else if (awpu_count == 1) {
            combinedFrame = bigFrames[0];
        } else {
            cv::hconcat(bigFrames[0], bigFrames[1], combinedFrame);
        }
        

        if (recording){// && !use_camera) {
            videoWriter.write(combinedFrame);// Write the frame to the video file
        }

        // Display the frame
        cv::imshow(APPLICATION_NAME, combinedFrame);

        // Wait for a key press and get the ASCII code
        int key = cv::waitKey(delay);// Wait for 10 ms

        // Check for specific keys
        switch (key) {
            case 'q':// Quit the application
                std::cout << "Quit key pressed ('q'). Exiting..." << std::endl;
                running = false;
                break;
            case 'r':// Respond to 'g' key
                if (recording) {
                    stopRecording(videoWriter);
                    recording = false;
                    std::cout << "Stopped recording" << std::endl;
                } else 
                {
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

    cv::destroyAllWindows();

    //for (auto& awpu: awpus) {
    //    delete awpu;
    //}

    return 0;
}