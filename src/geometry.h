#ifndef GEOMETRY_H
#define GEOMETRY_H
#include <Eigen/Dense>
#include <cmath>

#include "config.h"
#define X_INDEX 0
#define Y_INDEX 1
#define Z_INDEX 2
#define PI_HALF M_PI / 2.0

#define TO_RADIANS(degree) degree *(M_PI / 180.0)

#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

double clip(const double n, const double lower, const double upper);

double wrapAngle(const double angle);

double smallestAngle(const double target, const double current);


typedef Eigen::Vector3f Position;

/**
 * Convert spherical coordinates to cartesian coordinates
 */
Position spherical_to_cartesian(const double theta, const double phi, const double radius);


struct Spherical;
struct Horizontal;// Horizontal with a 90degree rotation on y-axis

inline double degrees(const double angle) {
    return angle * 180.0 / M_PI;
}

struct Direction {
    double theta, phi;

    Direction(){};
    Direction(double theta, double phi) : theta(theta), phi(phi){};
};

/**
 * This is the flipped spherical version where the [x, y, z] -> [x, z, y]
 */
struct Spherical {
    double theta, phi;

    Spherical(const Spherical &) = default;
    Spherical(Spherical &&) = default;
    Spherical &operator=(const Spherical &) & = default;
    Spherical &operator=(Spherical &&) & = default;

    Spherical() = default;
    Spherical(double theta, double phi) : theta(theta), phi(phi){};
    //Spherical(Spherical &&spherical) = default;
    //constexpr Spherical& operator=(const Spherical&);
    //{
    //  this->theta = spherical.theta;
    //  this->phi = spherical.phi;
    //};

    double distanceTo(const Horizontal &horizontal);
    double distanceTo(const Spherical &spherical);

    static Horizontal toHorizontal(const Spherical &spherical);
    static Position toCartesian(const Spherical &spherical, const double radius);

    //std::ostream &operator<<(std::ostream &out) {
    //    out << "Spherical: θ=" << degrees(theta) << " φ=" << degrees(phi);
    //    return out;
    //}

    //double distanceTo(const double theta, const double phi);
    //double distanceTo(const Spherical &direction);
    //Spherical directionTo(const Spherical &direction, const float step);

    friend std::ostream &operator<<(std::ostream &out, const Spherical &direction) {
        out << "Spherical: θ=" << degrees(direction.theta) << " φ=" << degrees(direction.phi);
        return out;
    }
};

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
        out << "Horizontal: azimuth=" << degrees(direction.azimuth) << " elevation=" << degrees(direction.elevation);
        return out;
    }
};

struct Cartesian {
    double x, y, z;
    Cartesian() : x(0), y(0), z(0){};
    Cartesian(double x, double y, double z) : x(x), y(y), z(z){};
    Cartesian(Cartesian &position) {
        x = position.x;
        y = position.y;
        z = position.z;
    };
    static Cartesian convert(const Spherical &spherical, const double radius);

    friend std::ostream &operator<<(std::ostream &out, const Cartesian &position) {
        out << "Cartesian: x=" << position.x << " y=" << position.y << " z=" << position.z;
        return out;
    }
};

#endif