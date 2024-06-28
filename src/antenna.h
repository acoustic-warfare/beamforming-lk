/** @file */

#ifndef ANTENNA_H
#define ANTENNA_H

#define X_INDEX 0
#define Y_INDEX 1
#define Z_INDEX 2

#include "config.h"
#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

typedef Vector3f Position;

/**
 * @brief Antenna that consists of points
 */
typedef struct {

  /**
   * The 3D representation of the antenna
   */
  MatrixXf points; // The 3D representation of the antenna
  int id;
} Antenna;

/**
 * Compute the delay in regard to the Z value
 */
VectorXf compute_delays(const MatrixXf &antenna);

/**
 * Find the center of antenna
 */
Position find_middle(const Antenna &antenna);

/**
 * Place the antenna by positioning the center @ new position
 */
void place_antenna(Antenna &antenna, const Position position);

/**
 * Generate a new antenna by specific options and place it ad the desired
 * position.
 */
Antenna create_antenna(const Position &position, const int columns,
                       const int rows, const float distance);

/**
 * Compute the delay in regard to the Z value. Since the antenna is being
 * thought as a point cloud laying on the XY plane, the Z value will be the
 * distance to the target and is used to calculate the necessary delays to
 * accomodate for a planar wave
 */
/**
 * @brief  Compute the delay in regard to the Z value. Since the antenna is
 * being thought as a point cloud laying on the XY plane, the Z value will be
 * the distance to the target and is used to calculate the necessary delays to
 * accomodate for a planar wave
 *
 * @param antenna The antenna that will be used to compute the delays
 * @return a 1D vector of time delays
 */
VectorXf compute_delays(const Antenna &antenna);

/**
 * Perform a rotation of the antenna in 3D space. This rotation consist of 3
 * rotations where the antenna must be projected onto the XY plane (Z values are
 * 0) (ZXZ extrinsic rotations). It is then rotated around the Z axis and
 * rotated around the X axis followed by the a negative rotation around the Z
 * axis to undo the first rotation. This "tilts" the rotation in 3D space while
 * still keeping each element in the viscinity of its initial position. No
 * elements have crossed paths.
 */
Antenna steer(const Antenna &antenna, const float phi, const float theta);

/**
 * Calculate the delays when antenna is steered towards angles theta and phi
 */
VectorXf steering_vector(const Antenna &antenna, float phi, float theta);

/**
 * Calculate the delays when antenna is steered towards a specific point located
 * on the unitsphere on the positive Z axis. A point may also not be located on
 * the unitsphere, however it must have a Z value >= 0
 */
VectorXf steering_vector(const Antenna &antenna, const Position point);

MatrixXf generate_unit_dome(const int n);

void generate_lookup_table(const MatrixXf &dome, MatrixXi &lookup_table);

void test_lookup_table(const MatrixXf &dome, const MatrixXi &lookup_table);

#endif
