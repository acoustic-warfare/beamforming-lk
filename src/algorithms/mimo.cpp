#include "mimo.h"

MIMOWorker::MIMOWorker(Pipeline *pipeline, Antenna &antenna, bool *running, int rows, int columns, float fov) : Worker(pipeline, antenna, running), rows(rows), columns(columns), fov(fov) {
    this->maxIndex = rows * columns;
    index = 0;
    this->powerdB = std::vector<float>(maxIndex, 0.0);
    computeDelayLUT();
    std::cout << "Created LUT from derived" << std::endl;
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
            //double z = 1.0;
            //double z = 0.0;
            double norm = sqrt(pow(x, 2) + pow(y, 2));

            x /= norm;
            y /= norm;
            //z /= norm;
            //double rang = sqrt(x*x + y*y);
            if (norm > 1.0) norm = 1.0;
            double theta = asin(norm);

            //double theta = acos(z);

            double phi = atan2(y, x);

            Eigen::VectorXf delays = steering_vector_spherical(antenna, theta, phi);
            //delays /= SAMPLE_RATE * 2000.0;
            //delays /= 2*M_PI;
            //for (int i = 0; i < ELEMENTS; i++) {
            //    float del = delays[i];
            //    double _offset;
            //    float fraction;
            //    fraction = static_cast<float>(modf((double) del, &_offset));
            //    int offset = N_SAMPLES - static_cast<int>(_offset);
            //    fractionalDelays[k][i] = fraction;
            //    offsetDelays[k][i] = offset;
            //}
            int i = 0;
            for (float del: delays) {
                double _offset;
                float fraction;
                fraction = static_cast<float>(modf((double) del, &_offset));
                int offset = N_SAMPLES - static_cast<int>(_offset);
                fractionalDelays[k][i] = fraction;
                offsetDelays[k][i] = offset;
                i++;
                //std::cout << "Creating LUT: " << k << " Mic: " << i<< std::endl;
            }

            k++;
        }
    }
}

void MIMOWorker::setup() {
    //computeDelayLUT();
    //std::cout << "Creating LUT" << std::endl;
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

    //std::cout << maxV << std::endl;

    //float lerp = (maxV*0.9 + minV*0.1) / 2.0
    int i = 0;
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < columns; c++) {
            double db = pow(powerdB[i] / maxV, 5);
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
    //computeDelayLUT();
    const float norm = 1.0 / static_cast<float>(antenna.usable);//.usable;
    int p = 0;
    float reference = 0.0;
    //float *signal = streams->get_signal(static_cast<unsigned>(antenna.index[10]), N_SAMPLES);


    float signals[ELEMENTS][N_ITEMS_BUFFER];
    for (int l = 0; l < antenna.usable; l++) {
        streams->read_stream(antenna.index[l], &signals[l][0]);
        //std::cout << "Reading stream: " << antenna.index[l] << ": Got: "<< signals[l][0]<<std::endl;
    }

    for (int l = 0; l < antenna.usable; l++) {
        for (int i = 0; i < N_SAMPLES; i++) {
            reference += powf(signals[antenna.index[l]][i + N_SAMPLES], 2);
        }
    }


    reference /= static_cast<float>(N_SAMPLES * antenna.usable);

    float powMax = 0.0;
    float powMin = 1000000.0;
    for (int m = 0; m < maxIndex; m++) {
        float out[N_SAMPLES] = {0.0};
        for (int s = 0; s < antenna.usable; s++) {
            int i = antenna.index[s];
            //std::cout << "Index: " << i <<" M="<<m<< std::endl;
            float fraction = fractionalDelays[m][i];
            //std::cout << "Fraction: " << fraction << std::endl;
            int offset = offsetDelays[m][i];
            //std::cout << "Offset: " << offset << " Of: "<<N_ITEMS_BUFFER<< std::endl;
            //std::cout << "Fraction: " << fraction << std::endl;
            //std::cout << "Value: " << signals[s][offset] << std::endl;// << fraction<<offset<<std::endl;

            //float *signal = this->streams->get_signal(static_cast<unsigned>(i), offset);
            delay(&out[0], &signals[s][offset], fraction);
        }
        float power = 0.0;
        for (int i = 0; i < N_SAMPLES; i++) {
            power += powf(out[i], 2);
        }

        power /= static_cast<float>(N_SAMPLES * antenna.usable);
        //if (power < powMin) powMin = power;
        //if (power > powMax) powMax = power;

        //std::cout << "Power: " << power << " Reference: " << reference << " Ratio: " << power / reference << std::endl;

        //if (power < reference*10.0) {
        //    power = 0.0;
        //}

        

        powerdB[m] = power;
    }
}