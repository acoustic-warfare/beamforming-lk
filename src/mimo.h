#ifndef MIMO_H
#define MIMO_H 


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

  float half_x = (float)(resolution_x) / 2 - 0.5;
  float half_y = (float)(resolution_y) / 2 - 0.5;
  int k = 0;
  for (int x = 0; x < resolution_x; x++) {
    for (int y = 0; y < resolution_y; y++) {

      // Imagine dome in spherical coordinates on the XY-plane with Z being
      // height
      float xo = (float)(x - half_x) / (resolution_x);
      float yo = (float)(y - half_y) / (resolution_y);
      float level = sqrt(xo * xo + yo * yo) / 1;
      level = sqrt(1 - level * level);
      Position point(xo, yo, level);
      // cout << point << endl;

      VectorXf tmp_delays = steering_vector(antenna, point);
      int i = 0;
      for (float del : tmp_delays) {
        double _offset;
        float fraction;

        fraction = (float)modf((double)del, &_offset);

        int offset = N_SAMPLES - (int)_offset;
        // cout << del << endl;
        fractional_delays[k * N_SENSORS + i] = fraction;
        offset_delays[k * N_SENSORS + i] = offset;
        i++;
      }

      k++;
    }
  }
}

#endif