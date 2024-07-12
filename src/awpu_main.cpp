#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "awpu.h"


int main() {

    std::cout << "Connecting to FPGA" << std::endl;
#if 0
    AWProcessingUnit awpu = AWProcessingUnit("127.0.0.1", 21844);
#else
    //AWProcessingUnit awpu = AWProcessingUnit("10.0.0.1", 21875);
    AWProcessingUnit awpu = AWProcessingUnit("10.0.0.1", 21878);
#endif

    std::cout << "Connected to FPGA" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    awpu.calibrate();
    //awpu5.calibrate();
    //awpu8.calibrate();

    std::cout << "Starting PSO" << std::endl;
    awpu.start(PSO);
    //awpu5.start(PSO);
    //awpu8.start(PSO);

    std::cout << "Starting listening" << std::endl;
    //awpu.resume();

    // Create a window to display the beamforming data
    cv::namedWindow(APPLICATION_NAME, cv::WINDOW_NORMAL);
    cv::resizeWindow(APPLICATION_NAME, APPLICATION_WIDTH, APPLICATION_HEIGHT);

    cv::Mat frame(Y_RES, X_RES, CV_8UC1);
    cv::Mat colorFrame(Y_RES, X_RES, CV_8UC1);


    while (1) {
        //awpu5.draw_heatmap(&frame);
        //awpu8.draw_heatmap(&frame);
        awpu.draw_heatmap(&frame);
        // Apply color map
        // Blur the image with a Gaussian kernel


        cv::GaussianBlur(frame, colorFrame,
                         cv::Size(BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE), 0);
        cv::applyColorMap(colorFrame, colorFrame, cv::COLORMAP_JET);
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