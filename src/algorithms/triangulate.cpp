//
// Created by janne on 2024-07-17.
//

#include "triangulate.h"
#include "nlohmann/json.hpp"
#include <opencv2/core/matx.hpp>

Eigen::Vector3d triangulatePoint(Eigen::ParametrizedLine<double, 3> &l1, Eigen::ParametrizedLine<double, 3> &l2) {
    using Vec3 = Eigen::Vector3d;

    const Vec3 p1 = l1.origin();
    const Vec3 p2 = l2.origin();
    Vec3 d1 = l1.direction();
    Vec3 d2 = l2.direction();

    Eigen::Matrix3d rotationMatrix;
    rotationMatrix << 1, 0, 0,
            0, 0, 1,
            0, 1, 0;

    d1 = rotationMatrix * d1;
    d2 = rotationMatrix * d2;

    const Vec3 w0 = p1 - p2;
    const double a = d1.dot(d1); // d1·d1
    const double b = d1.dot(d2); // d1·d2
    const double c = d2.dot(d2); // d2·d2
    const double d = d1.dot(w0); // d1·(p1 - p2)
    const double e = d2.dot(w0); // d2·(p1 - p2)

    const double denominator = a * c - b * b;
    double t, s;

    // Check if the lines are parallel
    if (std::abs(denominator) < 1e-6) {
        return Vec3::Zero();
    } else {
        // Lines are not parallel, calculate the nearest points
        t = (b * e - c * d) / denominator;
        s = (a * e - b * d) / denominator;
    }

    // Calculate the nearest points
    const Vec3 closestPoint1 = p1 + t * d1;
    const Vec3 closestPoint2 = p2 + s * d2;

    if((closestPoint1 - closestPoint2).norm() > 1) {
        return Vec3::Zero();
    }

    return (closestPoint1 + closestPoint2) / 2.0;
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
