#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "awpu.h"
#include "kf.h" 

KalmanFilter3D kf(0.2);

const Position start(3.0, 0.0, 1.4);


void point3d(const Spherical& spherical1, const Spherical& spherical2, const double separation) {
    double alpha = spherical1.phi - M_PI / 2.0;
    double beta = 3.0 * M_PI / 2.0 - spherical2.phi;
    double x = - separation*(sin(alpha) * sin(beta)) / sin(alpha + beta);

    double y = -x / tan(alpha);

    double alpha2 = M_PI / 2.0 - spherical1.theta;
    double beta2 = M_PI / 2.0 - spherical2.theta;

    double z = separation * sin(alpha2)*sin(beta2) / sin(alpha2 + beta2);

    Position point(y, z, x);
    point = point + start;

    kf.update(point);

    Position p = kf.getState();

    //std::cout <<" P0 " << spherical1 << std::endl;
    //std::cout << " P1 " << spherical2 << std::endl;

    std::cout << "Point: (" <<p(0) << ", " << p(1) << ", " << p(2) <<")" << std::endl;
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

#if 1
int main() {

    std::cout << "Connecting to FPGA" << std::endl;
#if 0
    AWProcessingUnit awpu = AWProcessingUnit("127.0.0.1", 21844);
#elif 0
    Pipeline pipeline(5000, Spherical(0, 0));
    AWProcessingUnit awpu5(&pipeline);
    AWProcessingUnit awpu8 = AWProcessingUnit("10.0.0.1", 21878);
#elif 0
    Pipeline pipeline("10.0.0.1", 21875);
    AWProcessingUnit awpu5 = AWProcessingUnit(&pipeline);
    AWProcessingUnit awpu8 = AWProcessingUnit(&pipeline);

#else
    AWProcessingUnit awpu5 = AWProcessingUnit("10.0.0.1", 21875);
    AWProcessingUnit awpu8 = AWProcessingUnit("10.0.0.1", 21878);
#endif

    std::cout << "Connected to FPGA" << std::endl;

    //std::cout << "Starting Gradient" << std::endl;

    //awpu5.start(GRADIENT);
    awpu5.start(MIMO);
    awpu8.start(MIMO);

    std::cout << "Starting listening" << std::endl;

    // Create a window to display the beamforming data
    cv::namedWindow(APPLICATION_NAME, cv::WINDOW_NORMAL);
    cv::resizeWindow(APPLICATION_NAME, APPLICATION_WIDTH*2, APPLICATION_HEIGHT);

    cv::Mat frame1(Y_RES, X_RES, CV_8UC1);
    cv::Mat frame2(Y_RES, X_RES, CV_8UC1);
    cv::Mat frame(Y_RES, X_RES*2, CV_8UC1);
    cv::Mat colorFrame(Y_RES, X_RES*2, CV_8UC1);

    cv::Mat small1(MIMO_SIZE, MIMO_SIZE, CV_8UC1);
    cv::Mat small2(MIMO_SIZE, MIMO_SIZE, CV_8UC1);


    while (1) {
        


        // Reset heatmap
        frame1.setTo(cv::Scalar(0));
        frame2.setTo(cv::Scalar(0));
        small1.setTo(cv::Scalar(0));
        small2.setTo(cv::Scalar(0));

#if 1
        awpu5.draw_heatmap(&frame1);
        //awpu8.draw_heatmap(&frame2);
        awpu5.draw_heatmap(&small1);
        awpu8.draw_heatmap(&small2);
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
        
        cv::hconcat(frame1,frame2,frame);

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

#else 

int main(int argc, char const *argv[])
{
    Spherical a(TO_RADIANS(90), TO_RADIANS(179));
    Spherical b(TO_RADIANS(90), TO_RADIANS(0));

    double theta = a.angle(b);

    std::cout << theta << std::endl;

    return 0;
}

#endif