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

Horizontal Spherical::toHorizontal(const Spherical &spherical) {
    Position position = spherical_to_cartesian(spherical.theta, spherical.phi, 1.0);
    double elevation = asin(position(Y_INDEX));
    double azimuth = asin(position(X_INDEX));

    return Horizontal(azimuth, elevation);
}


/**
 * Convert spherical coordinates to cartesian coordinates
 */
Position Spherical::toCartesian(const Spherical &spherical, const double radius = 1.0) {
    Position point;

    point(X_INDEX) = (float) (radius * (sin(spherical.theta) * cos(spherical.phi)));
    point(Y_INDEX) = (float) (radius * (sin(spherical.theta) * sin(spherical.phi)));
    point(Z_INDEX) = (float) (radius * (cos(spherical.theta)));

    return point;
}


Cartesian Cartesian::convert(const Spherical &spherical, const double radius = 1.0) {
    Cartesian point;

    point.x = (radius * (sin(spherical.theta) * cos(spherical.phi)));
    point.y = (radius * (sin(spherical.theta) * sin(spherical.phi)));
    point.z = (radius * (cos(spherical.theta)));

    return point;
}


//Spherical::Spherical(double theta, double phi, double radius)
//    : theta(theta), phi(phi), radius(radius) {}
//
//Position Spherical::toCartesian(const Spherical &spherical) {
//    Eigen::Vector3d cartesian;
//    cartesian(0) = spherical.radius * sin(spherical.theta) * cos(spherical.phi);
//    cartesian(1) = spherical.radius * sin(spherical.theta) * sin(spherical.phi);
//    cartesian(2) = spherical.radius * cos(spherical.theta);
//
//    return cartesian;
//}
//
//Eigen::Vector3d Spherical::toCartesian() {
//    Eigen::Vector3d cartesian;
//    cartesian(0) = radius * sin(theta) * cos(phi);
//    cartesian(1) = radius * sin(theta) * sin(phi);
//    cartesian(2) = radius * cos(theta);
//
//    return cartesian;
//}
//
///*Position Spherical::toCartesian() const {*/
///*  return Spherical::toCartesian(*this);*/
///*}*/
//
//Eigen::MatrixXd rotateTo(const Eigen::MatrixXd points, const double theta,
//                         const double phi) {
//    Eigen::Matrix3d Rz, Rx, Ry;
//
//    Rz << cos(phi), -sin(phi), 0.0, //
//            sin(phi), cos(phi), 0.0,//
//            0.0, 0.0, 1.0;
//
//    Ry << cos(theta), 0.0, sin(theta),//
//            0.0, 1.0, 0.0,            //
//            -sin(theta), 0.0, cos(theta);
//
//    Rx << 1.0, 0.0, 0.0,                 //
//            0.0, cos(theta), -sin(theta),//
//            0.0, sin(theta), cos(theta);
//
//    // Perform the rotation. Order of operations are important
//    Eigen::Matrix3d rotation = Ry * Rz;
//
//    Eigen::MatrixXd rotated = points * rotation;
//
//    return rotated;
//}
//
//std::vector<Spherical> Spherical::nearby(const double spread) {
//    Eigen::MatrixXd search(4, 3);
//
//    Eigen::Vector3d north, east, south, west;
//    constexpr double m = 0.1;
//    north = Spherical(TO_RADIANS(spread), TO_RADIANS(0.0)).toCartesian();
//    east = Spherical(TO_RADIANS(spread), TO_RADIANS(90.0)).toCartesian();
//    south = Spherical(TO_RADIANS(spread), TO_RADIANS(180.0)).toCartesian();
//    west = Spherical(TO_RADIANS(spread), TO_RADIANS(270.0)).toCartesian();
//
//    std::cout << east << std::endl;
//
//    std::cout << std::endl;
//
//    for (int i = 0; i < 3; i++) {
//        search(0, i) = north(i);
//        search(1, i) = east(i);
//        search(2, i) = south(i);
//        search(3, i) = west(i);
//    }
//
//    /*std::cout << north << east << south << west << std::endl;*/
//    // search.rows(0) = p;
//    /*search << north, east, south, west;*/
//
//    std::cout << search << std::endl;
//
//    search = rotateTo(search, theta, -phi);
//
//    std::cout << search << std::endl;
//
//    /*Spherical pnorth(theta - spread, phi);*/
//    /**/
//    /*if (pnorth.theta < 0.0) {*/
//    /*  pnorth.theta = -pnorth.theta;*/
//    /*  pnorth.phi = pnorth.phi + M_PI;*/
//    /*}*/
//    /**/
//    /*Spherical psouth(theta + spread, phi);*/
//    /*Spherical peast(theta, phi + spread);*/
//    /*Spherical pwest(theta, phi - spread);*/
//
//    /*Eigen::Vector3d northCart = north.toCartesian();*/
//    /*Eigen::Vector3d southCart = south.toCartesian();*/
//
//    std::vector<Spherical> near;
//
//    for (int i = 0; i < 4; i++) {
//        Eigen::Vector3d k = search.row(i);
//        /*k = k / k.norm();*/
//        double newTheta = acos(k.z());
//        std::cout << "i=" << i << std::endl
//                  << " " << k << std::endl;
//        /*std::cout << newTheta << std::endl << std::endl;*/
//        double newPhi = atan2(k.y(), k.x()) - M_PI;
//        near.emplace_back(newTheta, newPhi);
//    }
//
//    /*near.push_back(pnorth);*/
//    /*near.push_back(peast);*/
//    /*near.push_back(psouth);*/
//    /*near.push_back(pwest);*/
//
//    return near;
//}