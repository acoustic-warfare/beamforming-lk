#include "geometry.h"

double clip(const double n, const double lower, const double upper) {
    return std::max(lower, std::min(n, upper));
}

double wrapAngle(const double angle) {
    return fmod(angle, TWO_PI);
}

double smallestAngle(const double target, const double current) {
    return atan2(sin(target - current), cos(target - current));
}

/**
 * Convert spherical coordinates to cartesian coordinates
 */
Position spherical_to_cartesian(const double theta, const double phi, const double radius = 1.0) {
    Position point;

    point(X_INDEX) = (float) (radius * (sin(theta) * cos(phi)));
    point(Y_INDEX) = (float) (radius * (sin(theta) * sin(phi)));
    point(Z_INDEX) = (float) (radius * (cos(theta)));

    return point;
}

/**
 * Compute the distance between two spherical directions
 */
double Spherical::distanceTo(const Spherical &spherical) {
    return sqrt(
            2.0 - 2.0 * (sin(this->theta) * sin(spherical.theta) * cos(this->phi - spherical.phi) + cos(this->theta) * cos(spherical.theta)));
}

Spherical Horizontal::toSpherical(const Horizontal &horizontal) {
    double x = sin(horizontal.azimuth);
    double y = sin(horizontal.elevation);
    double phi = atan2(y, x);

    // We assume elevation 0 is horizontal
    double flipped_theta = PI_HALF - horizontal.elevation;

    // z
    double z_height = sin(flipped_theta) * cos(horizontal.azimuth);
    double theta = PI_HALF - asin(z_height);

    return Spherical(theta, phi);
}

Horizontal Spherical::toHorizontal(const Spherical &spherical) {
    Position position = spherical_to_cartesian(spherical.theta, spherical.phi, 1.0);
    double elevation = asin(position(Y_INDEX));
    double azimuth = asin(position(X_INDEX));

    return Horizontal(azimuth, elevation);
}


///**
// * Convert spherical coordinates to cartesian coordinates
// */
//Position Spherical::toCartesian(const Spherical &spherical, const double radius = 1.0) {
//    Position point;
//
//    point(X_INDEX) = (float) (radius * (sin(spherical.theta) * cos(spherical.phi)));
//    point(Y_INDEX) = (float) (radius * (sin(spherical.theta) * sin(spherical.phi)));
//    point(Z_INDEX) = (float) (radius * (cos(spherical.theta)));
//
//    return point;
//}


Cartesian Cartesian::convert(const Spherical &spherical, const double radius = 1.0) {
    Cartesian point;

    point.x = (radius * (sin(spherical.theta) * cos(spherical.phi)));
    point.y = (radius * (sin(spherical.theta) * sin(spherical.phi)));
    point.z = (radius * (cos(spherical.theta)));

    return point;
}