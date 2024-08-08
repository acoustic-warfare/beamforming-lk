/** @file triangulate.h
 * @author Janne
 * @brief Functions for triangulating points from 3D vectors and converting positions to GPS coordinates.
 * triangulate.h can currently only triangulate from data of two arrays. 
 * @date 2024-07-17
*/

#ifndef TRIANGULATE_H
#define TRIANGULATE_H

#include <gps.h>

#include <nlohmann/json_fwd.hpp>

#include "geometry.h"

/**
* @brief Finds an intersection between two vectors in 3D space.
* @param vector1 The first vector.
* @param vector2 The second vector.
* @param distance_threshold The maximum distance between the two closest points on the vectors. in metres.
* @returns The closest point between the two vectors. {0,0,0} if no intersection is found.
*
* This triangulation is used with real-life data, as such, no perfect triangulations can be achieved and some error is to be expected.
* Experiment with the distance yourself to try to find some parameters that tow the vector between false positives and false negatives.
*/
Eigen::Vector3d triangulatePoint(Eigen::ParametrizedLine<double, 3> &vector1, Eigen::ParametrizedLine<double, 3> &vector2,
                                 double distance_threshold = 1.0);

/**
 * @brief Finds an intersection between two vectors in 3D space, given their origin and direction vectors.
 * @param origin1 The origin of the first vector.
 * @param direction1 The direction of the first vector.
 * @param origin2 The origin of the second vector.
 * @param direction2 The direction of the second vector.
 * @param distance_threshold The maximum distance between the two closest points on the vectors, in meters. Default is 1.0.
 * @return The closest point between the two vectors. Returns {0,0,0} if no valid intersection is found.
 */
inline Eigen::Vector3d triangulatePoint(const Eigen::Vector3d &origin1, const Eigen::Vector3d &direction1, const Eigen::Vector3d &origin2,
                                        const Eigen::Vector3d &direction2, const double distance_threshold) {
    Eigen::ParametrizedLine<double, 3> vector1(origin1, direction1);
    Eigen::ParametrizedLine<double, 3> vector2(origin2, direction2);
    return triangulatePoint(vector1, vector2, distance_threshold);
}

/**
 * @brief Converts a 3D position to GPS coordinates.
 * @param position The 3D position to convert.
 * @param gps_position The reference GPS position to which the 3D position is relative.
 * @return A JSON object containing the GPS coordinates.
 */
nlohmann::json PositionToGPS(const Eigen::Vector3d &position, const gps_data_t &gps_position);

#endif//TRIANGULATE_H
