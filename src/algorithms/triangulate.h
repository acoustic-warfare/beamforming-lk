/** @file triangulate.h
 * @author Janne
 * @brief TODO:
 * @date 2024-07-17
*/

#ifndef BEAMFORMER_TRIANGULATE_H
#define BEAMFORMER_TRIANGULATE_H

#include <gps.h>
#include <nlohmann/json_fwd.hpp>

#include "../geometry.h"

/**
* @brief Finds an intersection between two lines in 3D space.
* @param l1 The first line.
* @param l2 The second line.
* @param distance_threshold The maximum distance between the two closest points on the lines. in metres.
* @returns The closest point between the two lines. {0,0,0} if no intersection is found.
*
* This triangulation is used with real-life data, as such, no perfect triangulations can be achieved and some error is to be expected.
* Experiment with the distance yourself to try to find some parameters that tow the line between false positives and false negatives.
*/
Eigen::Vector3d triangulatePoint(Eigen::ParametrizedLine<double, 3> &l1, Eigen::ParametrizedLine<double, 3> &l2,
                                 double distance_threshold = 1.0);

inline Eigen::Vector3d triangulatePoint(const Eigen::Vector3d &o1, const Eigen::Vector3d &d1, const Eigen::Vector3d &o2,
                                        const Eigen::Vector3d &d2, const double distance_threshold) {
    Eigen::ParametrizedLine<double, 3> line1(o1, d1);
    Eigen::ParametrizedLine<double, 3> line2(o2, d2);
    return triangulatePoint(line1, line2, distance_threshold);
}


nlohmann::json PositionToGPS(const Eigen::Vector3d &position, const gps_data_t &lk_position);

#endif //BEAMFORMER_TRIANGULATE_H
