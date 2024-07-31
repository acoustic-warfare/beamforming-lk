/** @file geometry.h
 * @author Irreq
 * @brief Geometric utilities and data structures for working with spherical, horizontal, and Cartesian coordinates.
 */

#ifndef BEAMFORMER_GEOMETRY_H
#define BEAMFORMER_GEOMETRY_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

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

#define EPSILON 1e-9

/**
 * @brief Clip value between two values.
 * @param n The value to be clipped.
 * @param lower The lower bound.
 * @param upper The upper bound.
 * @return The clipped value between lower and upper bounds.
 */
double clip(double n, double lower, double upper);

/**
 * @brief Wraps an angle to the range [0, 2π).
 * @param angle Angle in radians.
 * @return Wrapped angle in radians.
 */
double wrapAngle(double angle);

/**
 * @brief Retrieve the relative smallest angle between two angles.
 * @param target Target angle in radians.
 * @param current Current angle in radians.
 * @return Smallest angle between the target and current angles in radians.
 */
double smallestAngle(double target, double current);


typedef Eigen::Vector3f Position;

/**
 * @brief Rotation around Z-axis.
 * @param angle Angle of rotation in radians.
 * @return A 3x3 rotation matrix for rotation around the Z-axis.
 */
Eigen::Matrix3f rotateZ(float angle);

/**
 * @brief Rotation around Y-axis.
 * @param angle Angle of rotation in radians.
 * @return A 3x3 rotation matrix for rotation around the Y-axis.
 */
Eigen::Matrix3f rotateY(float angle);

/**
 * @brief Convert spherical coordinates to cartesian coordinates.
 * @param theta Inclination angle in radians.
 * @param phi Azimuth angle in radians.
 * @param radius Radius of the spherical coordinate system.
 * @return Cartesian coordinates as an `Eigen::Vector3f`.
 */
Position spherical_to_cartesian(double theta, double phi, double radius);


struct Spherical;
struct Horizontal;// Horizontal with a 90degree rotation on y-axis

/**
 * @brief Spherical representation of a position
 */
struct Spherical {
    /// Inclination angle in radians.
    double theta = 0;
    /// Azimuth angle in radians.
    double phi = 0;
    /// Radius of the spherical coordinate.
    double radius = 1.0;

    Spherical(const Spherical &) = default;
    Spherical(Spherical &&) = default;
    Spherical &operator=(const Spherical &) & = default;
    Spherical &operator=(Spherical &&) & = default;

    Spherical() = default;
    Spherical(double theta, double phi, double radius = 1.0);

    /**
     * @brief Computes the distance to another spherical coordinate.
     * @param spherical The other spherical coordinate.
     * @return The distance in radians.
     */
    double distanceTo(const Spherical &spherical) const;

    /**
     * @brief Finds nearby spherical coordinates within a given angular spread.
     * @param spread Angular spread in radians.
     * @return A vector of nearby spherical coordinates.
     */
    std::vector<Spherical> nearby(double spread);

    /**
     * @brief Computes the angular difference between this spherical coordinate and another.
     * @param spherical The other spherical coordinate.
     * @return The angular difference in radians.
     */
    double angle(const Spherical &spherical);

    /**
     * @brief Converts spherical coordinates to Cartesian coordinates.
     * @param spherical The spherical coordinates to convert.
     * @return Cartesian coordinates as an `Eigen::Vector3d`.
     */
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

    bool operator>(const double magnitude) const {
        return radius > magnitude;
    }

    bool operator<(const double magnitude) const {
        return radius < magnitude;
    }
};

/**
 * @brief Represents horizontal coordinates with azimuth and elevation angles.
 */
struct Horizontal {
    /// Azimuth angle in radians.
    double azimuth;
    /// Elevation angle in radians.
    double elevation;

    Horizontal() {};
    Horizontal(double azimuth, double elevation) : azimuth(azimuth), elevation(elevation) {};

    /**
     * @brief Converts horizontal coordinates to spherical coordinates.
     * @param horizontal The horizontal coordinates to convert.
     * @return Corresponding spherical coordinates.
     */
    static Spherical toSpherical(const Horizontal &horizontal);

    /**
     * @brief Computes the distance to another horizontal coordinate.
     * @param horizontal The other horizontal coordinate.
     * @return The distance in radians.
     */
    double distanceTo(const Horizontal &horizontal);

    /**
     * @brief Computes the distance to a spherical coordinate.
     * @param spherical The spherical coordinate.
     * @return The distance in radians.
     */
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
 * @brief Represents Cartesian coordinates with x, y, and z components.
 */
struct Cartesian {
    /// x-, y-, z-coordinates
    double x, y, z;

    Cartesian() : x(0), y(0), z(0) {};
    Cartesian(double x, double y, double z) : x(x), y(y), z(z) {};
    Cartesian(Cartesian &position) {
        x = position.x;
        y = position.y;
        z = position.z;
    };

    /**
     * @brief Converts spherical coordinates to Cartesian coordinates.
     * @param spherical The spherical coordinates to convert.
     * @param radius Radius of the spherical coordinate system.
     * @return Cartesian coordinates.
     */
    static Cartesian convert(const Spherical &spherical, double radius);

    friend std::ostream &operator<<(std::ostream &out, const Cartesian &position) {
        out << "Cartesian: x=" << position.x << " y=" << position.y << " z=" << position.z;
        return out;
    }
};

#endif //BEAMFORMER_GEOMETRY_H