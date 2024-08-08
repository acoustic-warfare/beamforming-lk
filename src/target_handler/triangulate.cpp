/** @file triangulate.cpp
 * @author Janne
 * @date 2024-07-17
*/

#include "triangulate.h"

#include "nlohmann/json.hpp"

Eigen::Vector3d triangulatePoint(Eigen::ParametrizedLine<double, 3> &vector1, Eigen::ParametrizedLine<double, 3> &vector2,
                                 const double distance_threshold) {
    using Vec3 = Eigen::Vector3d;

    // function for each vector (simple Y=kx+m): point = origin + direction * t (where t is the length along the vector)
    const Vec3 origin1 = vector1.origin();
    const Vec3 direction1 = vector1.direction();
    const Vec3 origin2 = vector2.origin();
    const Vec3 direction2 = vector2.direction();

    // the formula for finding the closest points between two lines is derived using alot of linera algebra but can be summed to:
    // n(tanget vector) = direction1 x direction2
    // t1 = (direction2 x n).(origin2 - origin1)/(n.n) (.=dot product, x=cross product)
    // use y=kx+m along the vector (closestPoint1 = origin1 + direction1 * t1)
    const Vec3 n = direction1.cross(direction2);// get tanget vector to calculate the closes point

    const double t1 = direction2.cross(n).dot(origin2 - origin1) / n.dot(n);// calculate the distance on the vector closest to the other vector
    const double t2 = direction1.cross(n).dot(origin2 - origin1) / n.dot(n);

    const Vec3 closestPoint1 = origin1 + direction1 * t1;// calculate the point from the function of the vector and the value t
    const Vec3 closestPoint2 = origin2 + direction2 * t2;

    if (constexpr double min_dist = 0, max_dist = 1;
        (closestPoint1 - closestPoint2).norm() > distance_threshold || ((closestPoint1 + closestPoint2) / 2).norm() > 20// We can't expect to hear targets further than 20 m away
        || (closestPoint1 + closestPoint2).z() < min_dist                                                               // Targets behind us are not valid
        || ((closestPoint1 + closestPoint2) / 2).z() < max_dist)                                                        // Targets too close are usually from static noise
    {
        return Vec3::Zero();
    }

    return (closestPoint1 + closestPoint2) / 2.0;// we return the point in the middle of the two closest points on the two direction vectors
}

nlohmann::json PositionToGPS(const Eigen::Vector3d &position, const gps_data_t &gps_position) {
    const double lat = gps_position.fix.latitude + position.x() / 111111.0;
    const double lon = gps_position.fix.longitude + position.y() / (111111.0 * std::cos(
                                                                                       gps_position.fix.latitude * M_PI / 180.0));
    const double alt = gps_position.fix.altitude + position.z();

    return {
            {"longitude", lon},
            {"latitude", lat},
            {"altitude", alt},
            {"type", "GeoPoint"}};
}
