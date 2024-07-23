//
// Created by janne on 2024-07-17.
//

#ifndef BEAMFORMER_TRIANGULATE_H
#define BEAMFORMER_TRIANGULATE_H

#include <gps.h>
#include <Eigen/Dense>
#include <nlohmann/json_fwd.hpp>

#include "../geometry.h"

Eigen::Vector3d triangulatePoint(Eigen::Vector3d v1, Eigen::Vector3d v2, double distance);

nlohmann::json PositionToGPS(const Eigen::Vector3d &position, const gps_data_t &lk_position);


#endif //BEAMFORMER_TRIANGULATE_H
