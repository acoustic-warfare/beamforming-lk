#include "miso.h"

MISOWorker::MISOWorker(Pipeline *pipeline, Antenna &antenna, bool *running, double fov) : Worker(pipeline, antenna, running), beamformer(antenna, pipeline->getStreams(), fov, TO_RADIANS(TRACKER_SPREAD)), aw(pipeline, &this->data[0]) {
    this->fov = TO_RADIANS(fov / 2.0);
    ratio = (sin(this->fov));
    beamformer.thetaLimit = fov;

    beamformer.move(Spherical(0, 0, 0));
    beamformer.startTracking(Spherical(0, 0, 0));
    thread_loop = std::thread(&MISOWorker::loop, this);
}

void MISOWorker::steer(Spherical direction) {
    beamformer.startTracking(direction);
#if DEBUG_MISO
    std::cout << "Steering to: " << direction << std::endl;
#endif
}

void MISOWorker::startTracking(Spherical direction) {
    beamformer.startTracking(direction);
}

void MISOWorker::update() {

    if (canContinue()) {
        float tmp_reference = 0.0;

        float *out = streams->get_signal(0, N_SAMPLES);
        for (int i = 1; i < N_SAMPLES - 1; i++) {
            float MA = out[i] * 0.5f - 0.25f * (out[i + 1] + out[i - 1]);
            tmp_reference += powf(MA, 2);
        }

        tmp_reference /= static_cast<float>(N_SAMPLES - 2);
        double reference = static_cast<double>(tmp_reference);

        for (int i = 0; i < 3; i++)
            beamformer.step(PARTICLE_RATE / 10, reference);
            
        beamformer.steer(beamformer.directionCurrent);
        beamformer.das(&data[0]);
    }
}

void MISOWorker::populateHeatmap(cv::Mat *heatmap) {
    double x_res = (double) heatmap->rows;
    double y_res = (double) heatmap->cols;

    Cartesian position = Cartesian::convert(beamformer.directionCurrent, 1.0);

    cv::Scalar color(255, 255, 255);

    // Use the position values to plot over the heatmap
    int x = (int) (x_res * (position.x / ratio / 2.0 + 0.5));
    int y = (int) (y_res * (position.y / ratio / 2.0 + 0.5));
    cv::circle(*heatmap, cv::Point(x, y_res - y - 1), 30, color, 2, 8, 0);

    
    if (beamformer.tracking) {
        std::stringstream ss;
        ss << "Tracking: ";
        cv::putText(*heatmap, ss.str(), cv::Point(0, y_res / 2), cv::FONT_HERSHEY_SIMPLEX, 1, color, 2);
    }
    
}