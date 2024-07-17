//
// Created by janne on 2024-07-17.
//

#ifndef BEAMFORMER_TRIANGULATE_H
#define BEAMFORMER_TRIANGULATE_H

#include <Eigen/Dense>
#include "../geometry.h"

Eigen::Vector2d calculateRelativePoint(const Eigen::Vector3d &a, const Eigen::Vector3d &b, double distance);

#endif //BEAMFORMER_TRIANGULATE_H
