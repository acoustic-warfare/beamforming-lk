#ifndef MIMO_H
#define MIMO_H

#include <Eigen/Dense>
#include <atomic>

#include "../antenna.h"
#include "../config.h"
#include "../pipeline.h"
#include "../streams.hpp"

#define VALID_SENSOR(i) (64 <= i) && (i < 128)


/**
 * @brief Calculate delays for different angles beforehand
 *
 * @param fractional_delays delays to use
 * @param antenna antenna structure
 * @param fov field of view
 * @param resolution_x width resolution
 * @param resolution_y height resolution
 */
void compute_scanning_window(int *offset_delays, float *fractional_delays,
                             const Antenna &antenna, float fov,
                             int resolution_x, int resolution_y) {

    float half_x = (float) (resolution_x) / 2 - 0.5;
    float half_y = (float) (resolution_y) / 2 - 0.5;
    int k = 0;
    for (int x = 0; x < resolution_x; x++) {
        for (int y = 0; y < resolution_y; y++) {

            // Imagine dome in spherical coordinates on the XY-plane with Z being
            // height
            float xo = (float) (x - half_x) / (resolution_x);
            float yo = (float) (y - half_y) / (resolution_y);
            float level = sqrt(xo * xo + yo * yo) / 1;
            level = sqrt(1 - level * level);
            Position point(xo, yo, level);
            // cout << point << endl;

            Eigen::VectorXf tmp_delays = steering_vector_cartesian(antenna, point);
            int i = 0;
            for (float del: tmp_delays) {
                double _offset;
                float fraction;

                fraction = (float) modf((double) del, &_offset);

                int offset = N_SAMPLES - (int) _offset;
                // cout << del << endl;
                fractional_delays[k * N_SENSORS + i] = fraction;
                offset_delays[k * N_SENSORS + i] = offset;
                i++;
            }

            k++;
        }
    }
}


/**
 * @brief Convert multiple input streams into single level by delay
 *
 * @param t_id [TODO:parameter]
 * @param task pool partition
 * @param fractional_delays delays to use
 * @param rb ring buffer to use
 * @return power level
 */
float miso(int t_id, int task, int *offset_delays, float *fractional_delays,
           Streams *streams) {
    float out[N_SAMPLES] = {0.0};
    int n = 0;
    for (int s = 0; s < N_SENSORS; s++) {

        // if (!((s == 64) || (s == 64 + 8) || (s == 127 - 16) || (s == 127))) {
        //   continue;
        // }

        if (VALID_SENSOR(s)) {
            float fraction = fractional_delays[s - 64];
            int offset = offset_delays[s - 64];

            float *signal = (float *) ((char *) streams->buffers[s] +
                                       streams->position + offset * sizeof(float));

            for (int i = 0; i < N_SAMPLES; i++) {
                out[i] += signal[i + 1] + fraction * (signal[i] - signal[i + 1]);
            }

            n++;
        }
    }

    float power = 0.f;
    float norm = 1 / (float) n;
    for (int p = 0; p < N_SAMPLES; p++) {

        power += powf(out[p] * norm, 2);
    }

    return power / (float) N_SAMPLES;
}


/**
 * Beamforming as fast as possible on top of pipeline
 */
void static_mimo_heatmap_worker(Pipeline *pipeline) {

    Antenna antenna = create_antenna(Position(0, 0, 0), COLUMNS, ROWS, DISTANCE);

    float fractional_delays[X_RES * Y_RES * N_SENSORS];
    int offset_delays[X_RES * Y_RES * N_SENSORS];

    compute_scanning_window(&offset_delays[0], &fractional_delays[0], antenna,
                            FOV, X_RES, Y_RES);

    int max = X_RES * Y_RES;

    float image[X_RES * Y_RES];

    int pixel_index = 0;

    int newData;
    float power;
    float threshold = 3e-8;

    float norm = 1 / 1e-05;

    float maxVal = 1.0;

    Streams *streams = pipeline->getStreams();

    while (pipeline->isRunning()) {

        // Wait for incoming data
        pipeline->barrier();

        // This loop may run until new data has been produced, meaning its up to
        // the machine to run as fast as possible
        newData = pipeline->mostRecent();
        float maxVal = 0.0;

        int i = 0;
        float mean = 0.0;

        int xi, yi = 0;
        float alpha = 1.0 / (float) (X_RES * Y_RES);
        alpha = 0.02;

        float heatmap_data[X_RES * Y_RES];

        float avgPower = 0.0;

        // Repeat until new data or abort if new data arrives
        while ((pipeline->mostRecent() == newData) && (i < max)) {
            //while ((i < max)) {

            int task = pixel_index * N_SENSORS;

            xi = pixel_index % X_RES;
            yi = pixel_index / X_RES;

            // Get power level from direction
            float val = miso(0, pixel_index, &offset_delays[task],
                             &fractional_delays[task], streams);

            if (val > maxVal) {
                maxVal = val;
            }

            // power = val * 1e5;

            // power = val * norm * 0.9f + 1.0;
            power = val + 1.0f;
            power = powf(power, 15);
            // power *= 1e9f;

            power = log(power) * 0.1f;

            power = power * norm * 0.9f;

            if (power < 0.2) {
                power = 0.0f;
            } else if (power > 1.0) {
                norm *= 0.95;
                // cout << "Bigger value" << endl;
                power = 1.0f;
            } else if (power < 0.0) {
                power = 0.0f;
                // cout << "Negative value" << endl;
            }

            // Paint pixel
            pipeline->magnitudeHeatmap->at<uchar>(yi, xi) = (uchar) (power * 255);

            pixel_index++;
            pixel_index %= X_RES * Y_RES;

            i++;
        }

        pipeline->canPlot = 1;

        norm = (1 - alpha) * norm + alpha * (1 / (maxVal));

        // norm = (1/maxVal) * 1.1f;

        // cout << maxVal << endl;
    }
}


#endif