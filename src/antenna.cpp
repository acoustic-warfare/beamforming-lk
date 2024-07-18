/*
 * A functional approach to Antenna computation for Digital Antenna Arrays (DAA)
 *
 * This file contains functions for creating, moving, rotating, combining DAA's
 *
 * We use the physics convention where theta \in [0, pi/2] and \phi \in [0, 2*pi)
 */

#include "antenna.h"

#include <iostream>

/**
 * Check if integer is in sector
 */
bool in_sector(const int *sector, const int i) {
    return (((sector[0] <= i) && (i <= sector[3])) ||
            ((sector[4] <= i) && (i <= sector[7])) ||
            ((sector[8] <= i) && (i <= sector[11])) ||
            ((sector[12] <= i) && (i <= sector[15])));
}

/**
 * Check if integer is in sector
 */
bool in_sector(const int sector_index, const int i) {
    const int *sector;

    switch (sector_index) {
        case 0:
            sector = &first_sector[0];
            break;
        case 1:
            sector = &second_sector[0];
            break;
        case 2:
            sector = &third_sector[0];
            break;
        default:
            sector = &fourth_sector[0];
            break;
    }

    return in_sector(sector, i);
}

/**
 * Spherical distance between two directions in spherical coordinates
 */
inline double spherical_distance(const double target_theta,
                                 const double target_phi,
                                 const double current_theta,
                                 const double current_phi) 
{
    double north_target_theta = PI_HALF - target_theta;
    double north_current_theta = PI_HALF - current_theta;
    return acos(sin(north_target_theta) * sin(north_current_theta) + cos(north_target_theta) * cos(north_current_theta) * cos(fabs(target_phi - current_phi)));
}



/**
 * @brief Helper function to return the middle point of the antenna
 *
 * @param antenna Antenna object
 * @return the point in 3D space of its center
 */
inline Position find_middle(const Antenna &antenna) {
    return antenna.points.colwise().mean();
}

/**
 * Place the antenna by positioning the center @ new position
 */
void place_antenna(Antenna &antenna, const Position position) {
    antenna.points.rowwise() += position.transpose() - find_middle(antenna).transpose();
}

/**
 * Generate a new antenna by specific options and place it ad the desired
 * position.
 */
Antenna create_antenna(const Position &position, const int columns,
                       const int rows, const float distance) {
    float half = distance / 2;
    Eigen::MatrixXf points(3, rows * columns);// (id, X|Y|Z)

    // Compute the positions of the antenna in 3D space
    int i = 0;
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < columns; c++) {
            points(X_INDEX, i) = (float)c * distance - rows * half + half;
            points(Y_INDEX, i) = (float)r * distance - columns * half + half;
            points(Z_INDEX, i) = 0.f;

            i++;
        }
    }


    Antenna antenna;

    antenna.id = 0;
    antenna.points = points;

    // Now we place the antenna for the user
    //place_antenna(antenna, position);

    return antenna;
}

/**
 * Compute the delay in regard to the Z value. Since the antenna is being
 * thought as a point cloud laying on the XY plane, the Z value will be the
 * distance to the target and is used to calculate the necessary delays to
 * accomodate for a planar wave
 */
Eigen::VectorXf compute_delays(const Antenna &antenna) {
    Eigen::VectorXf delays =
            antenna.points.row(Z_INDEX).array() * (SAMPLE_RATE / PROPAGATION_SPEED);

    // There is no need to delay the element that should be closest to source
    delays.array() -= delays.minCoeff();

    return delays;
}

/**
 * Perform a rotation of the antenna in 3D space. This rotation consist of 3
 * rotations where the antenna must be projected onto the XY plane (Z values are
 * 0) (ZXZ extrinsic rotations). It is then rotated around the Z axis and
 * rotated around the X axis followed by the a negative rotation around the Z
 * axis to undo the first rotation. This "tilts" the rotation in 3D space while
 * still keeping each element in the viscinity of its initial position. No
 * elements have crossed paths.
 */
inline Antenna steer(const Antenna &antenna, const double theta, const double phi) {

    // Perform the rotation. Order of operations are important
    Antenna rotated;
    rotated.points = (rotateY(-(float)theta) * (rotateZ((float)phi) * antenna.points));
    rotated.id = antenna.id;

    return rotated;
}

/**
 * Steer the antenna using horizontal angles. bore-sight is the x-axis and azimuth is the left-to right angles and elevation 
 * is up and down.
 */
Eigen::VectorXf steering_vector_horizontal(const Antenna &antenna, const double azimuth, const double elevation) {
    double x = sin(azimuth);
    double y = sin(elevation);
    double phi = atan2(y, x);
    double theta = PI_HALF - asin(1.0 - pow(x, 2) - pow(y, 2));

    Antenna steered = steer(antenna, theta, phi);
    return compute_delays(steered);
}

/**
 * Calculate the delays when antenna is steered towards a specific point located
 * on the unitsphere on the positive Z axis. A point may also not be located on
 * the unitsphere, however it must have a Z value >= 0
 */
Eigen::VectorXf steering_vector_cartesian(const Antenna &antenna, const Position &point) {
    double azimuth = atan2((double) point(Y_INDEX), (double) point(X_INDEX));
    double elevation = M_PI / 2.0f - asin((double) point(Z_INDEX));

    return steering_vector_horizontal(antenna, azimuth, elevation);
}

/**
 * Steer the antenna usin spherical coordinates where phi begins at Z+ axis
 */
Eigen::VectorXf steering_vector_spherical(const Antenna &antenna, const double theta, const double phi) {
    Antenna steered = steer(antenna, theta, phi);
    return compute_delays(steered);
}

/**
 * Steer the antenna usin spherical coordinates where phi begins at Z+ axis
 */
Eigen::VectorXf steering_vector_spherical(const Antenna &antenna, const Spherical &spherical) {
    Antenna steered = steer(antenna, spherical.theta, spherical.phi);
    return compute_delays(steered);
}


Eigen::MatrixXf generate_unit_dome(const int n) {
    Eigen::MatrixXf points(n, 3);// (id, X|Y|Z)

    double phi, theta;

    double magic = 2.0 * M_PI / 1.618033988749;

    for (int i = 0; i < n; i++) {
        phi = acos(1.0 - (double) i / double(n));
        theta = (double) i * magic;

        points(i, X_INDEX) = (float) (cos(theta) * sin(phi));
        points(i, Y_INDEX) = (float) (sin(theta) * sin(phi));
        points(i, Z_INDEX) = (float) (cos(phi));
    }

    return points;
}

void generate_lookup_table(const Eigen::MatrixXf &dome, Eigen::MatrixXi &lookup_table) {
    for (int phi = 0; phi < 90; ++phi) {
        for (int theta = 0; theta < 360; ++theta) {
            int best_match = -1;
            float min_dist = 1000000.0f;
            for (int i = 0; i < dome.size(); ++i) {
                float phi_radians = TO_RADIANS(float(phi));
                float theta_radians = TO_RADIANS(float(theta));
                float x = std::cos(theta_radians) * std::sin(phi_radians);
                float y = std::sin(theta_radians) * std::sin(phi_radians);
                float z = std::cos(phi_radians);

                auto dist = float(sqrt(
                        pow(x - dome(i, X_INDEX), 2) +
                        pow(y - dome(i, Y_INDEX), 2) +
                        pow(z - dome(i, Z_INDEX), 2)));

                best_match = dist < min_dist ? i : best_match;
                min_dist = dist < min_dist ? dist : min_dist;
            }
            lookup_table(phi, theta) = best_match;
        }
    }
}

void test_lookup_table(const Eigen::MatrixXf &dome, const Eigen::MatrixXi &lookup_table) {
    constexpr int TEST_CASES = 10000;
    constexpr float MAX_ALLOWED_DISTANCE = 0.2f;
    int failed_tests = 0;
    for (int i = 0; i < TEST_CASES; ++i) {
        int phi = rand() % 90;
        int theta = rand() % 360;

        int index = lookup_table(phi, theta);
        auto point = dome.row(index);

        float phi_radians = TO_RADIANS(double(phi));
        float theta_radians = TO_RADIANS(double(theta));
        float x = cos(theta_radians) * sin(phi_radians);
        float y = sin(theta_radians) * sin(phi_radians);
        float z = cos(phi_radians);

        auto dist = float(sqrt(
                pow(x - point(X_INDEX), 2) +
                pow(y - point(Y_INDEX), 2) +
                pow(z - point(Z_INDEX), 2)));

        if (dist > MAX_ALLOWED_DISTANCE) {
            failed_tests++;
            std::cout << "Test failed: " << dist << " > " << MAX_ALLOWED_DISTANCE << std::endl;
            std::cout << "Phi: " << phi << " Theta: " << theta << std::endl;
            std::cout << "Random point: " << x << " " << y << " " << z << std::endl;
            std::cout << "Looked up: " << point(X_INDEX) << " " << point(Y_INDEX) << " " << point(Z_INDEX) << std::endl;
        }
    }
    std::cout << "Completed " << TEST_CASES - failed_tests << "/" << TEST_CASES << " tests" << std::endl;
}
