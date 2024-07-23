#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "argparse/argparse.hpp"
#include "awpu.h"
#include "config.h"

#if 0
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "awpu.h"
#include "kf.h"

KalmanFilter3D kf(0.2);

const Position start(3.0, 0.0, 1.4);


void point3d(const Spherical &spherical1, const Spherical &spherical2, const double separation) {
    double alpha = spherical1.phi - M_PI / 2.0;
    double beta = 3.0 * M_PI / 2.0 - spherical2.phi;
    double x = -separation * (sin(alpha) * sin(beta)) / sin(alpha + beta);

    double y = -x / tan(alpha);

    double alpha2 = M_PI / 2.0 - spherical1.theta;
    double beta2 = M_PI / 2.0 - spherical2.theta;

    double z = separation * sin(alpha2) * sin(beta2) / sin(alpha2 + beta2);

    Position point(y, z, x);
    point = point + start;

    kf.update(point);

    Position p = kf.getState();

    //std::cout <<" P0 " << spherical1 << std::endl;
    //std::cout << " P1 " << spherical2 << std::endl;

    std::cout << "Point: (" << p(0) << ", " << p(1) << ", " << p(2) << ")" << std::endl;
}

void draw(cv::Mat &heatmap, Target &target) {
    double x_res = (double) heatmap.rows;
    double y_res = (double) heatmap.cols;

    // If we convert to sphere with radius 0.5 we don't have to normalize it other than add
    // 0.5 to get the sphere in the first sector in the cartesian coordinate system
    Cartesian position = Cartesian::convert(target.direction, 1.0);

    // Use the position values to plot over the heatmap
    int x = static_cast<int>(x_res * (position.x / 2.0 + 0.5));
    int y = static_cast<int>(y_res * (position.y / 2.0 + 0.5));

    int m = 255;

    cv::circle(heatmap, cv::Point(y, x), 5, cv::Scalar(m, m, m), cv::FILLED, 8, 0);
}

int main() {

    std::cout << "Connecting to FPGA" << std::endl;
#if 0
    AWProcessingUnit awpu = AWProcessingUnit("127.0.0.1", 21844);
#elif 0
    Pipeline pipeline(5000, Spherical(0, 0));
    AWProcessingUnit awpu5(&pipeline);
    AWProcessingUnit awpu8 = AWProcessingUnit("10.0.0.1", 21878);
#elif 1
    Pipeline pipeline5("10.0.0.1", 21875);
    AWProcessingUnit awpu51 = AWProcessingUnit(&pipeline5);
    AWProcessingUnit awpu52 = AWProcessingUnit(&pipeline5);

    Pipeline pipeline8("10.0.0.1", 21878);
    AWProcessingUnit awpu81 = AWProcessingUnit(&pipeline8);
    AWProcessingUnit awpu82 = AWProcessingUnit(&pipeline8);

#else
    AWProcessingUnit awpu5 = AWProcessingUnit("10.0.0.1", 21875);
    AWProcessingUnit awpu8 = AWProcessingUnit("10.0.0.1", 21878);
#endif

    std::cout << "Connected to FPGA" << std::endl;

    //std::cout << "Starting Gradient" << std::endl;

    awpu51.start(GRADIENT);
    awpu52.start(MIMO);

    awpu81.start(GRADIENT);
    awpu82.start(MIMO);

    std::cout << "Starting listening" << std::endl;

    // Create a window to display the beamforming data
    cv::namedWindow(APPLICATION_NAME, cv::WINDOW_NORMAL);
    cv::resizeWindow(APPLICATION_NAME, APPLICATION_WIDTH * 2, APPLICATION_HEIGHT);

    cv::Mat frame1(Y_RES, X_RES, CV_8UC1);
    cv::Mat frame2(Y_RES, X_RES, CV_8UC1);
    cv::Mat frame(Y_RES, X_RES * 2, CV_8UC1);
    cv::Mat colorFrame(Y_RES, X_RES * 2, CV_8UC1);

    cv::Mat small1(MIMO_SIZE, MIMO_SIZE, CV_8UC1);
    cv::Mat small2(MIMO_SIZE, MIMO_SIZE, CV_8UC1);

#if CAMERA
    cv::VideoCapture cap(CAMERA_PATH);// Open the default camera (change the
                                      // index if you have multiple cameras)
    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open the camera." << std::endl;
        //goto cleanup;
        exit(1);
    }

    cv::Mat cameraFrame;
#endif


    while (1) {


        // Reset heatmap
        frame1.setTo(cv::Scalar(0));
        frame2.setTo(cv::Scalar(0));
        small1.setTo(cv::Scalar(0));
        small2.setTo(cv::Scalar(0));

#if 1
        //awpu5.draw_heatmap(&frame1);
        //awpu8.draw_heatmap(&frame2);
        awpu52.draw_heatmap(&small1);
        awpu82.draw_heatmap(&small2);
#endif

        //for (Target &target : awpu5.targets()) {
        //    draw(frame1, target);
        //}
        //for (Target &target: awpu8.targets()) {
        //    draw(frame2, target);
        //}

        //std::cout << "Tracking: " << targets.size() << " objects" << std::endl;

        //for ()
        //awpu.draw_heatmap(&frame);
        //std::cout << "Best direction: " << awpu5.target() << std::endl;
        // Apply color map
        //cv::GaussianBlur(small, small,
        //                 cv::Size(BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE), 0);
        cv::resize(small1, frame1, frame1.size(), 0, 0, cv::INTER_LINEAR);
        cv::resize(small2, frame2, frame2.size(), 0, 0, cv::INTER_LINEAR);
        awpu51.draw_heatmap(&frame1);
        awpu81.draw_heatmap(&frame2);




        cv::hconcat(frame1, frame2, frame);

        // Blur the image with a Gaussian kernel
        cv::GaussianBlur(frame, colorFrame,
                         cv::Size(BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE), 0);
        cv::applyColorMap(colorFrame, colorFrame, cv::COLORMAP_JET);

        // Create a mask with the same dimensions as the image, initialized to black
        cv::Mat mask = cv::Mat::zeros(colorFrame.size(), colorFrame.type());

        // Define the circle parameters
        int radius = colorFrame.rows / 2;                                // Radius of the circles
        cv::Point center1(radius, colorFrame.rows / 2);                  // Center of the first circle
        cv::Point center2(colorFrame.cols - radius, colorFrame.rows / 2);// Center of the second circle

        // Draw filled white circles on the mask
        cv::circle(mask, center1, radius, cv::Scalar(255, 255, 255), -1);
        cv::circle(mask, center2, radius, cv::Scalar(255, 255, 255), -1);

        // Apply the mask to the original image
        cv::Mat result;
        colorFrame.copyTo(result, mask);

        colorFrame = result;

#if CAMERA
        cap >> cameraFrame;// Capture a frame from the camera

        if (cameraFrame.empty()) {
            std::cerr << "Error: Captured frame is empty." << std::endl;
            break;
        }

        cv::resize(cameraFrame, cameraFrame, cv::Size(colorFrame.cols, colorFrame.rows), 0, 0, cv::INTER_LINEAR);
        cv::addWeighted(cameraFrame, 1.0, colorFrame, 0.5, 0, colorFrame);
        //colorFrame = cameraFrame;
#endif
        cv::imshow(APPLICATION_NAME, colorFrame);
        if (cv::waitKey(1) == 'q') {
            std::cout << "Stopping application..." << std::endl;
            break;
        }
    }

    std::cout << "Pausing listening" << std::endl;

    //awpu5.pause();
    //awpu8.pause();
    //awpu.pause();

    std::cout << "Stopping PSO" << std::endl;

    //awpu.stop(PSO);

    //awpu5.stop(PSO);
    //awpu8.stop(PSO);

    std::cout << "Stopping program" << std::endl;

    return 0;
}

#elif 1

#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

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

//    // Get the frames per second of the video
//    double fps = cap.get(cv::CAP_PROP_FPS);
//
//    // Calculate the delay between frames
//    delay = 1000 / fps;
//
//
//    // Get the width and height of frames from the webcam
//    int frame_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
//    int frame_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
//    cv::Size frame_size(frame_width, frame_height);


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

    //program.add_argument("ports")
    //        .scan<'i', int>()
    //        .nargs(argparse::nargs_pattern::at_least_one)
    //        .help("PORT numbers");

    program.add_argument("--ports")
            //.default_value<std::vector<std::string>>({"orange"})
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
    std::vector<int> ports = program.get<std::vector<int>>("--ports");


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



    std::vector<AWProcessingUnit*> awpus;

    int i = 0;
    for (const int port : ports) {
        const char *addr = ip_address.c_str();
        //awpus.emplace_back(addr, port);
        //awpus[i].start(MIMO);
        //i++;
        std::cout << "Starting MIMO: " << port << std::endl;
        AWProcessingUnit* awpu = new AWProcessingUnit(ip_address.c_str(), port);
        awpu->start(MIMO);
        awpus.push_back(awpu);
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

    
    

    std::vector<cv::Mat> smallFrames;//(awpu_count, cv::Mat(mimo_res, mimo_res, CV_8UC1));
    std::vector<cv::Mat> bigFrames;//(awpu_count, cv::Mat(Y_RES, X_RES, CV_8UC1));

    //cv::Mat combinedFrame(Y_RES, X_RES * awpu_count, CV_8UC1);
    //cv::Mat colorFrame(Y_RES, X_RES * awpu_count, CV_8UC1);
    colorFrame.setTo(cv::Scalar(0));
    combinedFrame.setTo(cv::Scalar(0));

    //smallFrames.reserve(awpu_count);
    //bigFrames.reserve(awpu_count);
    for (int i = 0; i < awpu_count; i++) {
        //smallFrames.emplace_back(cv::Mat(mimo_res, mimo_res, CV_8UC1));
        smallFrames.push_back(cv::Mat(mimo_res, mimo_res, CV_8UC1));
        bigFrames.push_back(cv::Mat(Y_RES, X_RES, CV_8UC1));
    }
#if 0
    while (1) {
        for (int i = 0; i < awpu_count; i++) {
            smallFrames[i].setTo(cv::Scalar(0));
            //bigFrames[i].setTo(cv::Scalar(0));
            awpus[i]->draw_heatmap(&smallFrames[i]);
            cv::resize(smallFrames[i], bigFrames[i], bigFrames[i].size(), 0, 0, cv::INTER_LINEAR);
        }

        //cv::imshow(APPLICATION_NAME, smallFrames[0]);
        //awpus[0].draw_heatmap(&smallFrames[0]);
        //awpus[1].draw_heatmap(&smallFrames[1]);
        //cv::hconcat(smallFrames[0], smallFrames[1], combinedFrame);
        cv::hconcat(bigFrames[0], bigFrames[1], combinedFrame);
        //cv::imshow(APPLICATION_NAME, combinedFrame);

        // Blur the image with a Gaussian kernel
        cv::GaussianBlur(combinedFrame, colorFrame,
                         cv::Size(BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE), 0);
        cv::applyColorMap(colorFrame, colorFrame, cv::COLORMAP_JET);

        // Create a mask with the same dimensions as the image, initialized to black
        cv::Mat mask = cv::Mat::zeros(colorFrame.size(), colorFrame.type());

        // Define the circle parameters
        int radius = colorFrame.rows / 2;                                // Radius of the circles
        cv::Point center1(radius, colorFrame.rows / 2);                  // Center of the first circle
        cv::Point center2(colorFrame.cols - radius, colorFrame.rows / 2);// Center of the second circle

        // Draw filled white circles on the mask
        cv::circle(mask, center1, radius, cv::Scalar(255, 255, 255), -1);
        cv::circle(mask, center2, radius, cv::Scalar(255, 255, 255), -1);

        // Apply the mask to the original image
        cv::Mat result;
        colorFrame.copyTo(result, mask);

        colorFrame = result;

        cv::imshow(APPLICATION_NAME, colorFrame);
        if (cv::waitKey(1) == 'q') {
            std::cout << "Stopping application..." << std::endl;
            break;
        }
    }

    
#endif

    while (running) {

        cv::Mat frame;

        if (use_camera) {
            cap >> frame;// Capture a new frame from the webcam

            if (frame.empty()) {
                std::cerr << "Error: Could not read a frame from the webcam." << std::endl;
                break;
            }

            if (recording) {
                videoWriter.write(frame);// Write the frame to the video file
            }
        }


        for (int i = 0; i < awpu_count; i++) {
            smallFrames[i].setTo(cv::Scalar(0));
            //bigFrames[i].setTo(cv::Scalar(0));
            awpus[i]->draw_heatmap(&smallFrames[i]);
            cv::resize(smallFrames[i], bigFrames[i], bigFrames[i].size(), 0, 0, cv::INTER_LINEAR);

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
        

        if (recording && !use_camera) {
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
                    if (use_camera) {
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
    std::cout << "Recording stopped." << std::endl;

    for (auto& awpu: awpus) {
        delete awpu;
    }


    return 0;
}

#elif 0

int main(int argc, char const *argv[]) {
    Spherical a(TO_RADIANS(90), TO_RADIANS(179));
    Spherical b(TO_RADIANS(90), TO_RADIANS(0));

    double theta = a.angle(b);

    std::cout << theta << std::endl;

    return 0;
}

#endif