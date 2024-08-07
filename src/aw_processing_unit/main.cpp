#include <ctime>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>

#include "argparse/argparse.hpp"
#include "aw_processing_unit.h"

#define UDP_ADDRESS "10.0.0.1"

#define APPLICATION_NAME "Acoustic Warfare"
#define APPLICATION_WIDTH 1024
#define APPLICATION_HEIGHT 1024
#define RESOLUTION_MULTIPLIER 16
#define X_RES 1024
#define Y_RES 1024

#define BLUR_KERNEL_SIZE 11

#define BLUR_EFFECT true

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

/**
 * Setup argparse with different arguments
 */
void setupArgumentParser(argparse::ArgumentParser& program) {
    program.add_argument("--camera")
            .default_value(std::string("false"))
            .help("Camera option, default is false");

    program.add_argument("--ip-address")
            .default_value(std::string(UDP_ADDRESS))
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

    program.add_argument("--verbose")
            .default_value(false)
            .implicit_value(true)
            .help("Program output");

    program.add_argument("--fps")
            .default_value(false)
            .implicit_value(true)
            .help("Program FPS");

    program.add_argument("--logo")
            .default_value(false)
            .implicit_value(true)
            .help("Use logo");

    program.add_argument("--miso")
            .default_value(false)
            .implicit_value(true)
            .help("MISO Mode");

    program.add_argument("--debug")
            .default_value(false)
            .implicit_value(true)
            .help("Debug mode");

    program.add_argument("--fov")
            .default_value(static_cast<float>(180.0f))
            .scan<'g', float>()
            .help("Field of view");

    program.add_argument("--port")
            .append()
            .scan<'i', int>()
            .help("PORT numbers");
}

// Callback function to capture mouse events
void clickEvent(int event, int x, int y, int flags, void* userdata) {
    AWProcessingUnit *awpu = (AWProcessingUnit*)userdata;
    if (event == cv::EVENT_LBUTTONDOWN) {
        double xd = (static_cast<double>(x) / X_RES - 0.5) * 2.0;
        double yd = -(static_cast<double>(y) / Y_RES - 0.5) * 2.0;
        std::cout << "Coordinates: x=" << xd << ", y=" << yd << std::endl;
        double phi = atan2(yd, xd);
        double theta = sqrt(xd * xd + yd * yd);

        if (theta > 1.0) {
            theta = 1.0;
        }
        //theta *= M_PI / 2;
        theta = asin(theta);
        //double theta = (sqrt(2.0 - xd * xd - yd * yd) / 2.0);
        awpu->steer(Spherical(theta, phi));
    }
}


int main(int argc, char* argv[]) {
    argparse::ArgumentParser program(argv[0]);
    setupArgumentParser(program);
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
    bool verbose = program.get<bool>("--verbose");
    bool use_camera = (camera.compare("false") != 0);
    bool use_fps = program.get<bool>("--fps");
    bool use_logo = program.get<bool>("--logo");
    bool debug = program.get<bool>("--debug");
    bool miso = program.get<bool>("--miso");

    if (verbose) {
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
        std::cout << "Use camera: " << use_camera << std::endl;
    }


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

    // Setup AWPUS
    std::vector<AWProcessingUnit*> awpus;

    int i = 0;
    for (const int port: ports) {
        const char* addr = ip_address.c_str();
        if (verbose) {
            std::cout << "Starting MIMO: " << port << std::endl;
        }

        AWProcessingUnit* awpu = new AWProcessingUnit(ip_address.c_str(), port, fov, mimo_res, verbose, audio);
        awpus.push_back(awpu);

        // Start different modes
        if (tracking) { awpu->start(GRADIENT); }
        if (mimo) { awpu->start(MIMO); }
        if (miso) { awpu->start(MISO); }
    }

    int awpu_count = awpus.size();

    // Create a window to display the beamforming data
    cv::namedWindow(APPLICATION_NAME, cv::WINDOW_NORMAL);

    if (miso) {
        // Set the mouse callback function
        cv::setMouseCallback(APPLICATION_NAME, clickEvent, awpus[0]);
    }
    

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

    // Load the logo image (with alpha channel if available)
    cv::Mat logo;
    
    if (use_logo) {
      logo = cv::imread("logo.png", cv::IMREAD_UNCHANGED);// Load with alpha channel
      if (logo.empty()) {
        std::cerr << "Error: Unable to load logo image" << std::endl;
        return -1;
      }

        // Calculate new size (10% of original size)
        cv::Size newSize(static_cast<int>(logo.cols * 0.22), static_cast<int>(logo.rows * 0.22));

        // Resize the image
        cv::resize(logo, logo, newSize);
    }

    // Initialize variables for FPS calculation
    auto start = std::chrono::high_resolution_clock::now();
    int frameCount = 0;

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

            if (debug) {
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

        if (use_fps) {
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
        }


        if (use_logo) {
            int x = 840;
            int y = 0;
            // Convert logo to grayscale if it has an alpha channel or is in color
            if (logo.channels() == 4 || logo.channels() == 3) {
                cv::Mat logoGray;
                cv::cvtColor(logo, logoGray, cv::COLOR_BGR2GRAY);// Convert to grayscale
                cv::Mat logoColor;
                cv::applyColorMap(logoGray, logoColor, cv::COLORMAP_BONE);// Apply the same colormap if needed
                logoColor.copyTo(combinedFrame(cv::Rect(x, y, logoColor.cols, logoColor.rows)));
            } else if (logo.channels() == 1) {
                // Logo is already grayscale
                cv::Mat logoColor;
                cv::applyColorMap(logo, logoColor, cv::COLORMAP_BONE);// Apply colormap to logo if needed
                logoColor.copyTo(combinedFrame(cv::Rect(x, y, logoColor.cols, logoColor.rows)));
            } else {
                std::cerr << "Error: Unsupported number of channels in the logo image." << std::endl;
            }
        }

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

    for (auto& awpu: awpus) {
        delete awpu;
    }

    cv::destroyAllWindows();

    return 0;
}