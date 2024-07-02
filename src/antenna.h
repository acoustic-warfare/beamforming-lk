/** @file */

#ifndef ANTENNA_H
#define ANTENNA_H

#define X_INDEX 0
#define Y_INDEX 1
#define Z_INDEX 2

#define PI_HALF M_PI / 2.0

#define TO_RADIANS(degree) degree * (M_PI / 180.0)

#include "config.h"
#include <Eigen/Dense>
#include <cmath>


typedef Eigen::Vector3f Position;

/**
 * @brief Antenna that consists of points
 */
typedef struct {

  /**
   * The 3D representation of the antenna
   */
  Eigen::MatrixXf points; // The 3D representation of the antenna
  int id;
} Antenna;

//float to_radians(float degree);

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
Eigen::VectorXf compute_delays(const Antenna &antenna);

/**
 * Perform a rotation of the antenna in 3D space. This rotation consist of 3
 * rotations where the antenna must be projected onto the XY plane (Z values are
 * 0) (ZXZ extrinsic rotations). It is then rotated around the Z axis and
 * rotated around the X axis followed by the a negative rotation around the Z
 * axis to undo the first rotation. This "tilts" the rotation in 3D space while
 * still keeping each element in the viscinity of its initial position. No
 * elements have crossed paths.
 */
Antenna steer(const Antenna &antenna, const double theta, const double phi);

/**
 * Convert spherical coordinates to cartesian coordinates
 */
Position spherical_to_cartesian(const double theta, const double phi, const double radius);

/**
 * Steer the antenna using horizontal angles. bore-sight is the x-axis and azimuth is the left-to right angles and elevation
 * is up and down.
 */
Eigen::VectorXf steering_vector_horizontal(const Antenna &antenna, const double azimuth, const double elevation);

/**
 * Calculate the delays when antenna is steered towards a specific point located
 * on the unitsphere on the positive Z axis. A point may also not be located on
 * the unitsphere, however it must have a Z value >= 0
 */
Eigen::VectorXf steering_vector_cartesian(const Antenna &antenna, const Position &point);

/**
 * Steer the antenna usin spherical coordinates where phi begins at Z+ axis
 */
Eigen::VectorXf steering_vector_spherical(const Antenna &antenna, const double theta, const double phi);

Eigen::MatrixXf generate_unit_dome(const int n);

void generate_lookup_table(const Eigen::MatrixXf &dome, Eigen::MatrixXi &lookup_table);

void test_lookup_table(const Eigen::MatrixXf &dome, const Eigen::MatrixXi &lookup_table);

#endif
