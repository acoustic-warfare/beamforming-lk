/** @file mimo.cpp
 * @author Irreq
*/

#include "mimo.h"

MIMOWorker::MIMOWorker(Pipeline *pipeline, Antenna &antenna, bool *running, int rows, int columns, float fov) : Worker(pipeline, antenna, running), rows(rows), columns(columns), fov(fov) {
    this->maxIndex = rows * columns;
    index = 0;
    this->powerdB = std::vector<float>(maxIndex, 0.0);
    computeDelayLUT();
    thread_loop = std::thread(&MIMOWorker::loop, this);
}

void phaseLUT(double frequency) {
    double wavelength = PROPAGATION_SPEED / frequency;
    double k = 2 * M_PI / wavelength;
}

void MIMOWorker::computeDelayLUT() {
    double fovRadian = TO_RADIANS(fov);
    int k = 0;
    double separationRows = sin(fovRadian / 2.0) / (static_cast<double>(rows) / 2.0);
    double separationColumns = sin(fovRadian / 2.0) / (static_cast<double>(columns) / 2.0);

    offsetDelays = std::vector<std::vector<int>>(maxIndex, std::vector<int>(ELEMENTS, 0));
    fractionalDelays = std::vector<std::vector<float>>(maxIndex, std::vector<float>(ELEMENTS, 0.f));

    for (int r = 0; r < rows; r++) {

        for (int c = 0; c < columns; c++) {


            double y = static_cast<double>(r) * separationRows - static_cast<double>(rows) * separationRows / 2.0 + separationRows / 2.0;
            double x = static_cast<double>(c) * separationColumns - static_cast<double>(columns) * separationColumns / 2.0 + separationColumns / 2.0;
            double norm = sqrt(pow(x, 2) + pow(y, 2));

            x /= norm;
            y /= norm;
            if (norm > 1.0) norm = 1.0;
            double theta = asin(norm);

            double phi = atan2(y, x);

            int i = 0;
            for (float del: steering_vector_spherical(antenna, theta, phi)) {
                double _offset;
                float fraction;
                fraction = static_cast<float>(modf((double) del, &_offset));
                int offset = N_SAMPLES - static_cast<int>(_offset);
                fractionalDelays[k][i] = fraction;
                offsetDelays[k][i] = offset;
                i++;
            }

            k++;
        }
    }
}

void MIMOWorker::populateHeatmap(cv::Mat *heatmap) {
    float maxV = 0.0;
    float minV = 100000.0;

    for (auto &value: powerdB) {
        if (value > maxV) {
            maxV = value;
        }

        if (value < minV) {
            minV = value;
        }
    }

    float alpha = 0.2;
    prevPower = maxV * alpha + (1 - alpha) * prevPower;

    int i = 0;
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < columns; c++) {
#if USE_DB
            double db = pow((powerdB[i] - minV) / (maxV - minV), 3);
            std::cout << db << std::endl;
            db = 20 * log10(db * 1e12);
#else
            double db = pow(powerdB[i] / maxV, 5);
#endif
            db *= 255.0;
            db = clip(db, 0.0, 255.0);

            heatmap->at<uchar>(r, c) = static_cast<uchar>(db);
            i++;
        }
    }
}

void MIMOWorker::update() {
    const float norm = 1.0 / static_cast<float>(antenna.usable);//.usable;

    float signals[ELEMENTS][N_ITEMS_BUFFER];
    for (int l = 0; l < antenna.usable; l++) {
        streams->read_stream(antenna.index[l], &signals[l][0]);
    }

#if USE_REFERENCE
    float reference = 0.0;
    for (int l = 0; l < antenna.usable; l++) {
        for (int i = 0; i < N_SAMPLES; i++) {
            reference += powf(signals[antenna.index[l]][i + N_SAMPLES], 2);
        }
    }

    reference /= static_cast<float>(N_SAMPLES * antenna.usable);
#endif

    float powMax = 0.0;
    float powMin = 1000000.0;
    for (int m = 0; m < maxIndex; m++) {
        float out[N_SAMPLES] = {0.0};
        int count = 0;
        for (int s = 0; s < antenna.usable; s++) {
            int i = antenna.index[s];
            float fraction = fractionalDelays[m][i];
            int offset = offsetDelays[m][i];
            delay(&out[0], &signals[s][offset], fraction);
            count++;
        }
        float power = 0.0;
        
        for (int i = 1; i < N_SAMPLES-1 ; i++){
            float MA = out[i] * 0.5f - 0.25f*(out[i +1] + out[i -1]);
            power += powf(MA, 2);
        }

        power /= static_cast<float>(N_SAMPLES * count);

#if USE_REFERENCE
        if (power < reference * 10.0) {
            power = 0.0;
        }
#endif

        powerdB[m] = power;
    }
}
