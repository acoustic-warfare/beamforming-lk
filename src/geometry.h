/** @file geometry.h
 * @author Irreq
 * @brief TODO:
*/

#ifndef GEOMETRY_H
#define GEOMETRY_H
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "config.h"
#define X_INDEX 0
#define Y_INDEX 1
#define Z_INDEX 2
#define PI_HALF M_PI / 2.0

#define TO_RADIANS(degree) degree *(M_PI / 180.0)
#define TO_DEGREES(radian) radian * (180.0 / M_PI)

#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

/**
 * @brief Clip value between two values
 */
double clip(double n, double lower, double upper);

/**
 * @brief Modulo for radians
 */
double wrapAngle(double angle);

/**
 * @brief Retrieve the relative smallest angle between two angles
 */
double smallestAngle(double target, double current);


typedef Eigen::Vector3f Position;

/**
 * @brief Rotation around Z-axis
 */
Eigen::Matrix3f rotateZ(float angle);

/**
 * @brief Rotation around Y-axis
 */
Eigen::Matrix3f rotateY(float angle);

/**
 * @brief Convert spherical coordinates to cartesian coordinates
 */
Position spherical_to_cartesian(double theta, double phi, double radius);


struct Spherical;
struct Horizontal;// Horizontal with a 90degree rotation on y-axis

/**
 * @brief Spherical representation of a position
 */
struct Spherical {
    double theta = 0;// Inclination angle
    double phi = 0;  // Azimuth angle
    double radius = 1;

    Spherical(const Spherical &) = default;
    Spherical(Spherical &&) = default;
    Spherical &operator=(const Spherical &) & = default;
    Spherical &operator=(Spherical &&) & = default;

    Spherical() = default;
    Spherical(double theta, double phi, double radius = 1.0);

    double distanceTo(const Spherical &spherical) const;
    std::vector<Spherical> nearby(double spread);
    double angle(const Spherical &spherical);

    static Eigen::Vector3d toCartesian(const Spherical &spherical);
    [[nodiscard]] Eigen::Vector3d toCartesian() const;
    Eigen::Vector3d toCartesian();
    friend std::ostream &operator<<(std::ostream &out,
                                    const Spherical &direction) {
        out << "Spherical: θ=" << TO_DEGREES(wrapAngle(direction.theta))
            << "° φ=" << TO_DEGREES(wrapAngle(direction.phi))
            << "° r=" << direction.radius;
        return out;
    }
};

/**
 * @brief TODO:
 */
struct Horizontal {
    double azimuth, elevation;
    Horizontal(){};
    Horizontal(double azimuth, double elevation) : azimuth(azimuth), elevation(elevation){};

    static Spherical toSpherical(const Horizontal &horizontal);

    double distanceTo(const Horizontal &horizontal);
    double distanceTo(const Spherical &spherical);

    //std::ostream &operator<<(std::ostream &out) {
    //    out << "Horizontal: azimuth=" << degrees(azimuth) << " elevation=" << degrees(elevation);
    //    return out;
    //}

    friend std::ostream &operator<<(std::ostream &out, const Horizontal &direction) {
        out << "Horizontal: azimuth=" << TO_DEGREES(direction.azimuth) << " elevation=" << TO_DEGREES(direction.elevation);
        return out;
    }
};

/**
 * @brief TODO:
 */
struct Cartesian {
    double x, y, z;
    Cartesian() : x(0), y(0), z(0){};
    Cartesian(double x, double y, double z) : x(x), y(y), z(z){};
    Cartesian(Cartesian &position) {
        x = position.x;
        y = position.y;
        z = position.z;
    };
    static Cartesian convert(const Spherical &spherical, double radius);

    friend std::ostream &operator<<(std::ostream &out, const Cartesian &position) {
        out << "Cartesian: x=" << position.x << " y=" << position.y << " z=" << position.z;
        return out;
    }
};

#endif