#include "mimo.h"

MIMOWorker::MIMOWorker(Pipeline *pipeline, Antenna &antenna, bool *running, int rows, int columns, float fov) : Worker(pipeline, antenna, running), antenna(antenna), rows(rows), columns(columns), fov(fov) {
    maxIndex = rows * columns;
    index = 0;
    this->powerdB = std::vector<float>(maxIndex, 0.0);
    //this->angleLUT = std::vector<std::vector<Spherical>>(rows, std::vector<Spherical>(columns, Spherical(0,0)));
    computeDelayLUT();
    this->streams = pipeline->getStreams();

    //for (auto &b : offsetDelays) {
    //    for (auto &a : b)
    //        std::cout << a << " ";
    //}

    //std::cout << std::endl;
    //std::cout << antenna.usable << std::endl;
}

void MIMOWorker::computeDelayLUT() {
    double fovRadian = TO_RADIANS(fov);
    int k = 0;
    double separationRows = sin(fovRadian / 2.0) / ((double)rows / 2.0);
    double separationColumns = sin(fovRadian / 2.0) / ((double)columns / 2.0);

    this->offsetDelays = std::vector<std::vector<int>>(maxIndex, std::vector<int>(ELEMENTS, 0));
    this->fractionalDelays = std::vector<std::vector<float>>(maxIndex, std::vector<float>(ELEMENTS, 0.f));

    for (int r = 0; r < rows; r++) {
        
        for (int c = 0; c < columns; c++) {

            
            double y = (double)r * separationRows - (double)rows * separationRows / 2.0 + separationRows / 2.0;
            double x = (double) c * separationColumns - (double) columns * separationColumns / 2.0 + separationColumns / 2.0;
            double z = 1.0;
            double norm = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));

            x /= norm;
            y /= norm;
            z /= norm;

            double theta = acos(z);

            double phi = atan2(y, x);

            //Spherical spherical(theta, phi);

            Eigen::VectorXf delays = steering_vector_spherical(antenna, theta, phi);
            int i = 0;
            for (float del: delays) {

                double _offset;
                float fraction;
                fraction = (float) modf((double) del, &_offset);
                int offset = N_SAMPLES - (int) _offset;
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

    for (auto &value : powerdB) {
        if (value > maxV) {
            maxV = value;
        }

        if (value < minV) {
            minV = value;
        }
    }

    //float lerp = (maxV*0.9 + minV*0.1) / 2.0
    int i = 0;
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < columns; c++) {
            double db = pow((powerdB[i] - minV) / (maxV - minV), 3);
            //std::cout << db << std::endl;
            //db = 20 * log10(db * 1e12);
            db *= 255.0;
            db = clip(db, 0.0, 255.0);

            //if (db < 100) {
            //    db = 0;
            //}
            //std::cout << db << std::endl;
            heatmap->at<uchar>(r, c) = (uchar)db;//(255.f * clip(db, 0.0, 1.0));
            i++;
        }
    }
}

void MIMOWorker::update() {
    const float norm = 1 / (float) antenna.usable;//.usable;
    int p = 0;
    float reference = 0.0;
    float *signal = this->streams->get_signal(0, 0);
    for (int i = 0; i < N_SAMPLES; i++) {
        reference += powf(signal[i], 2);
    }

    reference /= (float)N_SAMPLES;
    while (p < maxIndex && canContinue() ) {
        float out[N_SAMPLES] = {0.0};

        for (unsigned s = 0; s < antenna.usable; s++) {
            if (!in_sector(&second[0], s)) {
                continue;
            }
            int i = antenna.index[s];
            float fraction = fractionalDelays[index][i];
            int offset = offsetDelays[index][i];

            float *signal = this->streams->get_signal((unsigned)i, offset);
            delay(&out[0], signal, fraction);
        }
        float power = 0.0;
        for (int i = 0; i < N_SAMPLES; i++) {
            power += powf(out[i], 2) * norm;
        }

        power /= (float) N_SAMPLES;

        powerdB[index] = power / reference;//clip(power - reference, 0.0, power);
        index++;
        index %= maxIndex;
        p++;
    }
}