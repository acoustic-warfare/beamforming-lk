//
// Created by janne on 2024-07-17.
//

#include "triangulate.h"
#include "nlohmann/json.hpp"
#include <opencv2/core/matx.hpp>

Eigen::Vector3d triangulatePoint(Eigen::Vector3d v1, Eigen::Vector3d v2, const double distance) {
    using Vec3 = Eigen::Vector3d;

    // Matrix to rotate the vectors from a lying down perspective (z "outwards") to a standing up perspective (z upwards)
    const Eigen::Matrix3d rotationMatrix = (Eigen::Matrix3d() << 0, 0, 1,
                                            1, 0, 0,
                                            0, 1, 0).finished();
    v1 = rotationMatrix * v1;
    v2 = rotationMatrix * v2;


    const Vec3 a_start(distance / 2, 0, 0);
    const Vec3 b_start(-distance / 2, 0, 0);

    const Vec3 u = v1.normalized();
    const Vec3 v = v2.normalized();
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

    if ((closest_point_a - closest_point_b).norm() > 1)
        return Vec3::Zero();

    // Midpoint of the closest approach
    Vec3 midpoint = (closest_point_a + closest_point_b) / 2.0;

    if (midpoint.x() < 0)
        midpoint *= -1;

    return midpoint;
}

nlohmann::json PositionToGPS(const Eigen::Vector3d &position, const gps_data_t &lk_position) {
    const double lat = lk_position.fix.latitude + position.x() / 111111.0;
    const double lon = lk_position.fix.longitude + position.y() / (111111.0 * std::cos(
                                                                       lk_position.fix.latitude * M_PI / 180.0));
    const double alt = lk_position.fix.altitude + position.z();

    return {
        {"longitude", lon},
        {"latitude", lat},
        {"altitude", alt},
        {"type", "GeoPoint"}
    };
}
