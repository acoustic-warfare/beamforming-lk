/** @file triangulate.cpp
 * @author Janne
 * @date 2024-07-17
*/

#include "triangulate.h"
#include "nlohmann/json.hpp"


Eigen::Vector3d triangulatePoint(Eigen::ParametrizedLine<double, 3> &l1, Eigen::ParametrizedLine<double, 3> &l2,
                                 const double distance_threshold) {
    using Vec3 = Eigen::Vector3d;

    const Vec3 r1 = l1.origin();
    const Vec3 e1 = l1.direction();
    const Vec3 r2 = l2.origin();
    const Vec3 e2 = l2.direction();

    const Vec3 n = e1.cross(e2);
    const double t1 = e2.cross(n).dot(r2 - r1) / n.dot(n);
    const double t2 = e1.cross(n).dot(r2 - r1) / n.dot(n);

    const Vec3 closestPoint1 = r1 + e1 * t1;
    const Vec3 closestPoint2 = r2 + e2 * t2;

    if (constexpr double min_dist = 0, max_dist = 1;
        (closestPoint1 - closestPoint2).norm() > distance_threshold
        || ((closestPoint1 + closestPoint2) / 2).norm() > 20 // We can't expect to hear targets further than 20 m away
        || (closestPoint1 + closestPoint2).z() < min_dist // Targets behind us are not valid
        || ((closestPoint1 + closestPoint2) / 2).z() < max_dist) {
        // Targets too close are usually from static noise
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
