/*
 * A functional approach to Antenna computation for Digital Antenna Arrays (DAA)
 *
 * This file contains functions for creating, moving, rotating, combining DAA's
 *
 */

#include "antenna.h"

using namespace Eigen;
using namespace std;

/**
 * @brief Convert degree to radians
 *
 * @param degree The angle to be converted
 * @return angle as radian
 */
inline float to_radians(float degree) { return degree * (M_PI / 180.0); }

/**
 * @brief Helper function to return the middle point of the antenna
 *
 * @param antenna Antenna object
 * @return the point in 3D space of its center
 */
inline Position find_middle(const Antenna &antenna) {
  return antenna.points.colwise().mean();
  // return antenna.points.rowwise().mean();
}

// ********** Antenna in space **********

/**
 * Place the antenna by positioning the center @ new position
 */
void place_antenna(Antenna &antenna, const Position position) {
  antenna.points.rowwise() +=
      position.transpose() - find_middle(antenna).transpose();
  // antenna.points.colwise() += position - find_middle(antenna);
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
  // MatrixXf points(3, rows * columns); // (id, X|Y|Z)
  //
  // // Compute the positions of the antenna in 3D space
  // int i = 0;
  // for (int y = 0; y < columns; y++) {
  //   for (int x = 0; x < rows; x++) {
  //     points(X_INDEX, i) = x * distance - rows * half + half;
  //     points(Y_INDEX, i) = y * distance - columns * half + half;
  //     points(Z_INDEX, i) = 0.f;
  //
  //     i++;
  //   }
  // }

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
  // VectorXf delays =
  //     antenna.points.row(Z_INDEX).array() * (SAMPLE_RATE /
  //     PROPAGATION_SPEED);
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
Antenna steer(const Antenna &antenna, const float phi, const float theta) {
  Matrix3f Rz1, Rx, Rz2;

  Rz1 << cos(phi), -sin(phi), 0, sin(phi), cos(phi), 0, 0, 0, 1;
  Rx << 1, 0, 0, 0, cos(theta), -sin(theta), 0, sin(theta), cos(theta);
  Rz2 << cos(-phi), -sin(-phi), 0, sin(-phi), cos(-phi), 0, 0, 0, 1;

  // Perform the rotation. Order of operations are important
  Matrix3f rotation = Rz2 * (Rx * Rz1);

  Antenna rotated;
  rotated.points = antenna.points * rotation;
  rotated.id = antenna.id;

  return rotated;
}

/**
 * Calculate the delays when antenna is steered towards angles theta and phi
 */
VectorXf steering_vector(const Antenna &antenna, float phi, float theta) {
  Antenna steered = steer(antenna, phi, theta);
  VectorXf delays = compute_delays(steered);

  return delays;
}
/**
 * Calculate the delays when antenna is steered towards a specific point located
 * on the unitsphere on the positive Z axis. A point may also not be located on
 * the unitsphere, however it must have a Z value >= 0
 */
VectorXf steering_vector(const Antenna &antenna, const Position point) {
  float theta, phi;

  phi = atan2(point(Y_INDEX), point(X_INDEX));
  theta = M_PI / 2 - asin(point(Z_INDEX));

  return steering_vector(antenna, phi, theta);
}

#if 0
#include <iostream>
int main() {
  // Test the antenna
  Vector3f position(0, 0, 0);

  // cout << create_antenna(position, COLUMNS, ROWS, DISTANCE) << endl;
}
#endif

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
