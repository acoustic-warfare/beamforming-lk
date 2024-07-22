//
// Created by janne on 2024-07-17.
//

#include "triangulate.h"

#include <opencv2/core/matx.hpp>

Eigen::Vector3d calculateRelativePoint(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const double distance) {
    using Vec3 = Eigen::Vector3d;

    const Vec3 a_start(-distance / 2, 0, 0);
    const Vec3 b_start(distance / 2, 0, 0);

    const Vec3 u = a.normalized();
    const Vec3 v = b.normalized();
    const Vec3 w0 = a_start - b_start;

    const double a_dot_a = u.dot(u);
    const double b_dot_b = v.dot(v);
    const double a_dot_b = u.dot(v);
    const double a_dot_w0 = u.dot(w0);
    const double b_dot_w0 = v.dot(w0);

    const double denominator = a_dot_a * b_dot_b - a_dot_b * a_dot_b;

    if (std::abs(denominator) < 1e-6) {
        return Vec3::Zero();
    }

    const double ta = (a_dot_b * b_dot_w0 - b_dot_b * a_dot_w0) / denominator;
    const double tb = (a_dot_a * b_dot_w0 - a_dot_b * a_dot_w0) / denominator;

    const Vec3 closest_point_a = a_start + ta * u;
    const Vec3 closest_point_b = b_start + tb * v;

    // Midpoint of the closest approach
    Vec3 midpoint = (closest_point_a + closest_point_b) / 2.0;

    return midpoint;
}

nlohmann::json PositionToGPS(const Eigen::Vector3d &position, const gps_data_t &lk_position) {

}
