//
// Created by janne on 2024-07-17.
//

#ifndef BEAMFORMER_TRIANGULATE_H
#define BEAMFORMER_TRIANGULATE_H

#include <gps.h>
#include <Eigen/Dense>
#include <nlohmann/json_fwd.hpp>

#include "../geometry.h"

Eigen::Vector2d calculateRelativePoint(const Eigen::Vector3d &a, const Eigen::Vector3d &b, double distance);

nlohmann::json PositionToGPS(const Eigen::Vector2d &position, gps_data_t, const gps_data_t &lk_position);


#endif //BEAMFORMER_TRIANGULATE_H
