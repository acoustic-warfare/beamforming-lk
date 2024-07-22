#include "mimo.h"

MIMOWorker::MIMOWorker(Pipeline *pipeline, Antenna &antenna, bool *running, int rows, int columns, float fov) : Worker(pipeline, antenna, running), rows(rows), columns(columns), fov(fov) {
    this->maxIndex = rows * columns;
    index = 0;
    this->powerdB = std::vector<float>(maxIndex, 0.0);
    this->computeDelayLUT();
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
            double z = 1.0;
            double norm = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));

            x /= norm;
            y /= norm;
            z /= norm;

            double theta = acos(z);

            double phi = atan2(y, x);

            Eigen::VectorXf delays = steering_vector_spherical(antenna, theta, phi);
            int i = 0;
            for (float del: delays) {

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

void MIMOWorker::setup() {
    this->computeDelayLUT();
    std::cout << "Creating LUT" << std::endl;
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
    float alpha = 0.2;
    prevPower = maxV * alpha + (1-alpha) * prevPower;

    //std::cout << maxV << std::endl;

    //float lerp = (maxV*0.9 + minV*0.1) / 2.0
    int i = 0;
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < columns; c++) {
            double db = pow(powerdB[i] / maxV, 3);
            //double db = pow((powerdB[i] - minV) / (maxV - minV), 3);//(maxV - minV), 3);
            //std::cout << db << std::endl;
            //db = 20 * log10(db * 1e12);
            db *= 255.0;
            db = clip(db, 0.0, 255.0);

            //if (db < 100) {
            //    db = 0;
            //}
            //std::cout << db << std::endl;
            heatmap->at<uchar>(r, c) = static_cast<uchar>(db);//(255.f * clip(db, 0.0, 1.0));
            i++;
        }
    }
}

void MIMOWorker::update() {
    const float norm = 1.0 / static_cast<float>(antenna.usable);//.usable;
    int p = 0;
    float reference = 0.0;
    float *signal = this->streams->get_signal(static_cast<unsigned>(antenna.index[0]), 0);
    for (int i = 0; i < N_SAMPLES; i++) {
        reference += powf(signal[i], 2);
    }

    reference /= static_cast<float>(N_SAMPLES);

    float signals[ELEMENTS][N_ITEMS_BUFFER];
    for (int l = 0; l < antenna.usable; l++) {
        streams->read_stream(antenna.index[l], &signals[l][0]);
        //std::cout << "Reading stream: " << antenna.index[l] << ": Got: "<< signals[l][0]<<std::endl;
    }

    
    //computeDelayLUT();
    //std::cout << "Working" << std::endl;
    for (int m = 0; m < maxIndex; m++) {
        float out[N_SAMPLES] = {0.0};
        for (unsigned s = 0; s < antenna.usable; s++) {
            int i = antenna.index[s];
            //std::cout << "Index: " << i << std::endl;
            float fraction = fractionalDelays[m][i];
            //std::cout << "Fraction: " << fraction << std::endl;
            int offset = offsetDelays[m][i];
            //std::cout << "Offset: " << offset << std::endl;

            //std::cout << "Value: " << signals[s][N_SAMPLES] << " and: " << signals[s][i] << fraction<<offset<<std::endl;

            //float *signal = this->streams->get_signal(static_cast<unsigned>(i), offset);
            delay(&out[0], &signals[s][N_SAMPLES + offset], fraction);
        }
        float power = 0.0;
        for (int i = 0; i < N_SAMPLES; i++) {
            power += powf(out[i] * norm, 2);
        }

        power /= static_cast<float>(N_SAMPLES);

        //if (power < reference * 0.01) {
        //    power = 0.0;
        //}

        //std::cout << "Power: " << power << std::endl;

        powerdB[m] = power;
    }

//    while (p < maxIndex && canContinue() ) {
//        float out[N_SAMPLES] = {0.0};
//
//        for (unsigned s = 0; s < antenna.usable; s++) {
//            //if (!in_sector(&second[0], s)) {
//            //    continue;
//            //}
//            int i = antenna.index[s];
//            float fraction = fractionalDelays[index][i];
//            int offset = offsetDelays[index][i];
//
//            float *signal = this->streams->get_signal(static_cast<unsigned>(i), offset);
//            delay(&out[0], signal, fraction);
//        }
//        float power = 0.0;
//        for (int i = 0; i < N_SAMPLES; i++) {
//            power += powf(out[i] * norm, 2);
//        }
//
//        power /= static_cast<float>(N_SAMPLES);
//
//        //if (power < reference * 0.01) {
//        //    power = 0.0;
//        //}
//
//        powerdB[index] = power;// / reference;//clip(power - reference, 0.0, power);
//        index++;
//        index %= maxIndex;
//        p++;
//    }
}