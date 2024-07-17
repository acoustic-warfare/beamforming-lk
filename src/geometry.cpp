#include "geometry.h"

double clip(const double n, const double lower, const double upper) {
    return std::max(lower, std::min(n, upper));
}

double wrapAngle(const double angle) {

    double result = fmod(angle, 2.0 * M_PI);

    if (result < 0.0) {
        return 2.0 * M_PI + result;
    } else {
        return result;
    }
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

Cartesian Cartesian::convert(const Spherical &spherical, const double radius = 1.0) {
    Cartesian point;

    point.x = (radius * (sin(spherical.theta) * cos(spherical.phi)));
    point.y = (radius * (sin(spherical.theta) * sin(spherical.phi)));
    point.z = (radius * (cos(spherical.theta)));

    return point;
}


Spherical::Spherical(double theta, double phi, double radius)
    : theta(theta), phi(phi), radius(radius) {}

Eigen::Vector3d Spherical::toCartesian(const Spherical &spherical) {
    Eigen::Vector3d cartesian;
    cartesian(0) = spherical.radius * sin(spherical.theta) * cos(spherical.phi);
    cartesian(1) = spherical.radius * sin(spherical.theta) * sin(spherical.phi);
    cartesian(2) = spherical.radius * cos(spherical.theta);

    return cartesian;
}

Eigen::Vector3d Spherical::toCartesian() {
    Eigen::Vector3d cartesian;
    cartesian(0) = radius * sin(theta) * cos(phi);
    cartesian(1) = radius * sin(theta) * sin(phi);
    cartesian(2) = radius * cos(theta);

    return cartesian;
}

Eigen::Vector3d Spherical::toCartesian() const {
    Eigen::Vector3d cartesian;
    cartesian(0) = radius * sin(theta) * cos(phi);
    cartesian(1) = radius * sin(theta) * sin(phi);
    cartesian(2) = radius * cos(theta);

    return cartesian;
}

double Spherical::angle(const Spherical &spherical) {
    // Function to compute the geodesic distance between two points on a sphere
    double sin_theta1 = sin(M_PI / 2.0 - theta);
    double sin_theta2 = sin(M_PI / 2.0 - spherical.theta);
    double cos_theta1 = cos(M_PI / 2.0 - theta);
    double cos_theta2 = cos(M_PI / 2.0 - spherical.theta);
    double delta_phi = phi - spherical.phi;

    return acos(sin_theta1 * sin_theta2 + cos_theta1 * cos_theta2 * cos(delta_phi));
}

Eigen::MatrixXd rotateTo(const Eigen::MatrixXd points, const double theta,
                         const double phi) {
    Eigen::Matrix3d Rz, Rx, Ry;

    Rz << cos(phi), -sin(phi), 0.0, //
            sin(phi), cos(phi), 0.0,//
            0.0, 0.0, 1.0;

    Ry << cos(theta), 0.0, sin(theta),//
            0.0, 1.0, 0.0,            //
            -sin(theta), 0.0, cos(theta);

    Rx << 1.0, 0.0, 0.0,                 //
            0.0, cos(theta), -sin(theta),//
            0.0, sin(theta), cos(theta);

    // Perform the rotation. Order of operations are important
    Eigen::Matrix3d rotation = Ry * Rz;

    Eigen::MatrixXd rotated = points * rotation;

    return rotated;
}

std::vector<Spherical> Spherical::nearby(const double spread) {
    Eigen::MatrixXd search(4, 3);

    Eigen::Vector3d north = Spherical(spread, TO_RADIANS(0.0)).toCartesian();
    Eigen::Vector3d east  = Spherical(spread, TO_RADIANS(90.0)).toCartesian();
    Eigen::Vector3d south = Spherical(spread, TO_RADIANS(180.0)).toCartesian();
    Eigen::Vector3d west  = Spherical(spread, TO_RADIANS(270.0)).toCartesian();

    for (int i = 0; i < 3; i++) {
        search(0, i) = north(i);
        search(1, i) = east(i);
        search(2, i) = south(i);
        search(3, i) = west(i);
    }

    double rotateTheta = theta;

    if (rotateTheta + spread > M_PI / 2.0) {
        rotateTheta -= spread;
        theta -= spread / 2.0;
    }


    search = rotateTo(search, rotateTheta, phi);

    std::vector<Spherical> near;

    for (int i = 0; i < 4; i++) {
        Eigen::Vector3d k = search.row(i);
        /*k = k / k.norm();*/
        double newTheta = acos(k.z());
        double newPhi = atan2(k.y(), k.x()) - M_PI;
        near.emplace_back(newTheta, newPhi);
    }

    return near;
}

Eigen::Matrix3f rotateZ(const float angle) {
    Eigen::Matrix3f Rz;
    Rz << (float) cos((double)angle), -(float) sin((double)angle), 0.0f,
            (float) sin((double)angle), (float) cos((double)angle), 0.0f,
            0.0f, 0.0f, 1.0f;
    return Rz;
}

Eigen::Matrix3f rotateY(const float angle) {
    Eigen::Matrix3f Ry;
    Ry << (float) cos((double)angle), 0.0, (float) sin((double)angle),//
            0.0, 1.0, 0.0,                            //
            -(float) sin((double)angle), 0.0, (float) cos((double)angle);
    return Ry;
}