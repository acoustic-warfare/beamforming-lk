/** @file antenna.h
 * @author Irreq
 * @brief Antenna computation for Digital Antenna Arrays (DAA)
 * This file contains functions for creating, moving, rotating, combining DAA's
 * We use the physics convention where theta in [0, pi/2] and phi in [0, 2*pi)
 */

#ifndef ANTENNA_H
#define ANTENNA_H

#include <Eigen/Dense>
#include <cmath>

#include "geometry.h"

#define PROPAGATION_SPEED 340.0// Speed of sound in air [m * s^-1]
#define SAMPLE_RATE 48828.0    // Hz [s^-1]
#define COLUMNS 8
#define ROWS 8
#define ELEMENTS (COLUMNS * ROWS)
#define DISTANCE 0.02// [m]

const int second[16] = {
        0, 2, 4, 6,    // first column
        16, 18, 20, 22,//
        32, 34, 36, 38,//
        48, 50, 52, 54

};


const int second_sector[16] = {0, 1, 2, 3, //
8, 9, 10, 11, //
16, 17, 18, 19, //
24, 25, 26, 27};

const int first_sector[16] = {4, 5, 6, 7, //
                              12, 13, 14, 15, //
                              20, 21, 22, 23, //
                              28, 29, 30, 31};

const int third_sector[16] = {32, 33, 34, 35,//
                              40, 41, 42, 43,//
                              48, 49, 50, 51,//
                              56, 57, 58, 59};

const int fourth_sector[16] = {36, 37, 38, 39,//
                               44, 45, 46, 47,//
                               52, 53, 54, 55,//
                               60, 61, 62, 63};

/**
 * @brief Check if integer is in sector.
 * @param sector The sector array to check against.
 * @param i The index to check.
 * @return True if the index is in the sector, false otherwise.
 */
bool in_sector(const int *sector, const int i);

/**
 * @brief Check if integer is in sector.
 * @param sector_index The index of the sector to check against.
 * @param i The index to check.
 * @return True if the index is in the sector, false otherwise.
 */
bool in_sector(const int sector_index, const int i);

/**
 * @brief TODO:
 */
struct Sector {
    int sector;// 1, 2, 3, 4
    int usable = 0;
    int index[16];
};

/**
 * @brief Antenna that consists of points
 */
struct Antenna {

    /**
     * @brief The 3D representation of the antenna
     */
    Eigen::MatrixXf points;      // The 3D representation of the antenna
    int id;                      // Id of antenna
    int usable = 0;              ///< Number of usable elements;
    int *index;                  ///< Index of usable element
    float *power_correction_mask;///< Correction of microphone to reach some value
    float mean;                  ///< Pseudo valid metrics for checking the power level of antenna during auto calibration
    float median;

    // TODO: calculate angle correction
    float *angle_correction_mask;///< How much to compensate for non-isotropic elements
    int angle_resolution;

    ~Antenna() {
        if (usable > 0) {
            delete[] index;
            delete[] power_correction_mask;
        }
    }
};

/**
 * @brief Find the center of the antenna.
 * @param antenna The antenna whose center is to be found.
 * @return The position representing the center of the antenna.
 */
Position find_middle(const Antenna &antenna);

/**
 * @brief Place the antenna by positioning the center @ new position.
 * @param antenna The antenna to be placed.
 * @param position The new position for the center of the antenna.
 */
void place_antenna(Antenna &antenna, const Position position);

/**
 * @brief Generate a new antenna by specific options and place it ad the desired
 * position.
 * @param position The desired position for the new antenna.
 * @param columns The number of columns in the antenna array.
 * @param rows The number of rows in the antenna array.
 * @param distance The distance between elements in the antenna array.
 * @return The created antenna.
 */
Antenna create_antenna(const Position &position, const int columns,
                       const int rows, const float distance);

/**
 * @brief Compute the delay in regard to the Z value. Since the antenna is being
 * thought as a point cloud laying on the XY plane, the Z value will be the
 * distance to the target and is used to calculate the necessary delays to
 * accomodate for a planar wave.
 * @param antenna The antenna that will be used to compute the delays
 * @return a 1D vector of time delays
 */
Eigen::VectorXf compute_delays(const Antenna &antenna);

/**
 * @brief Perform a rotation of the antenna in 3D space. This rotation consist of 3
 * rotations where the antenna must be projected onto the XY plane (Z values are
 * 0) (ZXZ extrinsic rotations). It is then rotated around the Z axis and
 * rotated around the X axis followed by the a negative rotation around the Z
 * axis to undo the first rotation. This "tilts" the rotation in 3D space while
 * still keeping each element in the viscinity of its initial position. No
 * elements have crossed paths.
 * @param antenna The antenna to be rotated.
 * @param theta The angle of rotation around the X axis.
 * @param phi The angle of rotation around the Z axis.
 * @return The rotated antenna.
 */
Antenna steer(const Antenna &antenna, const double theta, const double phi);

/**
 * @brief Steer the antenna using horizontal angles. bore-sight is the x-axis and 
 * azimuth is the left-to right angles and elevation is up and down.
 * @param antenna The antenna to be steered.
 * @param azimuth The azimuth angle for steering.
 * @param elevation The elevation angle for steering.
 * @return A 1D vector representing the steering vector.
 */
Eigen::VectorXf steering_vector_horizontal(const Antenna &antenna, const double azimuth, const double elevation);

/**
 * @brief Calculate the delays when antenna is steered towards a specific point located
 * on the unitsphere on the positive Z axis. A point may also not be located on
 * the unitsphere, however it must have a Z value >= 0.
 * @param antenna The antenna to be steered.
 * @param point The target point for steering.
 * @return A 1D vector representing the steering vector.
 */
Eigen::VectorXf steering_vector_cartesian(const Antenna &antenna, const Position &point);

/**
 * @brief Steer the antenna using spherical coordinates where phi begins at Z+ axis.
 * @param antenna The antenna to be steered.
 * @param theta The polar angle for steering.
 * @param phi The azimuthal angle for steering.
 * @return A 1D vector representing the steering vector.
 */
Eigen::VectorXf steering_vector_spherical(const Antenna &antenna, const double theta, const double phi);

/**
 * @brief Steer the antenna using spherical coordinates where phi begins at Z+ axis.
 * @param antenna The antenna to be steered.
 * @param spherical The spherical coordinates for steering.
 * @return A 1D vector representing the steering vector.
 */
Eigen::VectorXf steering_vector_spherical(const Antenna &antenna, const Spherical &spherical);

/**
 * @brief Generate a unit dome with specified resolution.
 * @param n The resolution of the unit dome.
 * @return A matrix representing the generated unit dome.
 */
Eigen::MatrixXf generate_unit_dome(const int n);

/**
 * @brief Generate a lookup table for the unit dome.
 * @param dome The unit dome matrix.
 * @param lookup_table The generated lookup table.
 */
void generate_lookup_table(const Eigen::MatrixXf &dome, Eigen::MatrixXi &lookup_table);

void test_lookup_table(const Eigen::MatrixXf &dome, const Eigen::MatrixXi &lookup_table);

#endif//ANTENNA_H
