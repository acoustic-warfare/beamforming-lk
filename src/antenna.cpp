/*
 * A functional approach to Antenna computation for Digital Antenna Arrays (DAA)
 *
 * This file contains functions for creating, moving, rotating, combining DAA's
 *
 */

#include "antenna.h"

/**
 * @brief Convert degree to radians
 *
 * @param degree The angle to be converted
 * @return angle as radian
 */
float to_radians(float degree) { return degree * (M_PI / 180.0); }

/**
 * @brief Helper function to return the middle point of the antenna
 *
 * @param antenna Antenna object
 * @return the point in 3D space of its center
 */
inline Position find_middle(const Antenna &antenna) {
  return antenna.points.colwise().mean();
}

// ********** Antenna in space **********

/**
 * Place the antenna by positioning the center @ new position
 */
void place_antenna(Antenna &antenna, const Position position) {
  antenna.points.rowwise() += position.transpose() - find_middle(antenna).transpose();
}

/**
 * Generate a new antenna by specific options and place it ad the desired
 * position.
 */
Antenna create_antenna(const Position &position, const int columns,
                       const int rows, const float distance) {
  float half = distance / 2;
  MatrixXf points(rows * columns, 3); // (id, X|Y|Z)

  // Compute the positions of the antenna in 3D space
  int i = 0;
  for (int y = 0; y < columns; y++) {
    for (int x = 0; x < rows; x++) {
      points(i, X_INDEX) = x * distance - rows * half + half;
      points(i, Y_INDEX) = y * distance - columns * half + half;
      points(i, Z_INDEX) = 0.f;

      i++;
    }
  }

  Antenna antenna;

  antenna.id = 0;
  antenna.points = points;

  // Now we place the antenna for the user
  place_antenna(antenna, position);

  return antenna;
}

/**
 * Compute the delay in regard to the Z value. Since the antenna is being
 * thought as a point cloud laying on the XY plane, the Z value will be the
 * distance to the target and is used to calculate the necessary delays to
 * accomodate for a planar wave
 */
VectorXf compute_delays(const Antenna &antenna) {
  VectorXf delays =
      antenna.points.col(Z_INDEX).array() * (SAMPLE_RATE / PROPAGATION_SPEED);

  // There is no need to delay the element that should be closest to source
  delays.array() -= delays.minCoeff();

  return delays;
}

/**
 * Perform a rotation of the antenna in 3D space. This rotation consist of 3
 * rotations where the antenna must be projected onto the XY plane (Z values are
 * 0) (ZXZ extrinsic rotations). It is then rotated around the Z axis and
 * rotated around the X axis followed by the a negative rotation around the Z
 * axis to undo the first rotation. This "tilts" the rotation in 3D space while
 * still keeping each element in the viscinity of its initial position. No
 * elements have crossed paths.
 */
Antenna steer(const Antenna &antenna, const float azimuth, const float elevation) {
  Matrix3f Rz1, Rx, Rz2;

  Rz1 << cos(azimuth), -sin(azimuth), 0, sin(azimuth), cos(azimuth), 0, 0, 0, 1;
  Rx << 1, 0, 0, 0, cos(elevation), -sin(elevation), 0, sin(elevation), cos(elevation);
  Rz2 << cos(-azimuth), -sin(-azimuth), 0, sin(-azimuth), cos(-azimuth), 0, 0, 0, 1;

  // Perform the rotation. Order of operations are important
  Matrix3f rotation = Rz2 * (Rx * Rz1);

  Antenna rotated;
  rotated.points = antenna.points * rotation;
  rotated.id = antenna.id;

  return rotated;
}



/**
 * Convert spherical coordinates to cartesian coordinates
 */
Position spherical_to_cartesian(const float theta, const float phi, const float radius = 1.0f) {
  Position point;

  point(X_INDEX) = radius * (float)(cos(theta) * sin(phi));
  point(Y_INDEX) = radius * (float)(sin(theta) * sin(phi));
  point(Z_INDEX) = radius * (float)(cos(phi));

  return point;
}


#if 0 // TODO
Position horizontal_to_cartesian(const float azimuth, const float elevation, const float radius = 1.0f) {
  Position point;

  point(X_INDEX) = radius * (float)cos((double))
}
#endif



VectorXf steering_vector_horizontal(const Antenna &antenna, float azimuth, float elevation) {
  Antenna steered = steer(antenna, azimuth, elevation);
  return compute_delays(steered);
}

/**
 * Calculate the delays when antenna is steered towards a specific point located
 * on the unitsphere on the positive Z axis. A point may also not be located on
 * the unitsphere, however it must have a Z value >= 0
 */
VectorXf steering_vector_cartesian(const Antenna &antenna, const Position &point) {
  float azimuth = (float)atan2((double)point(Y_INDEX), (double)point(X_INDEX));
  float elevation = (float)(M_PI / 2.0f - asin((double)point(Z_INDEX)));

  return steering_vector_horizontal(antenna, azimuth, elevation);
}

VectorXf steering_vector_spherical(const Antenna &antenna, const float theta, const float phi) {
  Position point = spherical_to_cartesian(theta, phi);

  return steering_vector_cartesian(antenna, point);
}

#if 0
#include <iostream>
int main() {
  // Test the antenna
  Vector3f position(0, 0, 0);

  // cout << create_antenna(position, COLUMNS, ROWS, DISTANCE) << endl;
}


MatrixXf generate_unit_dome(const int n) {
  MatrixXf points(n, 3); // (id, X|Y|Z)

  double phi, theta;

  double magic = 2.0 * M_PI / 1.618033988749;

  for (int i = 0; i < n; i++) {
    phi = acos(1.0 - (double)i / double(n));
    theta = (double)i * magic;

    points(i, X_INDEX) = (float)(cos(theta) * sin(phi));
    points(i, Y_INDEX) = (float)(sin(theta) * sin(phi));
    points(i, Z_INDEX) = (float)(cos(phi));
  }

  return points;
}

#endif
