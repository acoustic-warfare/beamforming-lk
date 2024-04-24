# cython: language_level=3
# distutils: language=c++

"""
File: eigen.pxd 
Author: Irreq 
Date: 2024-04-06

Description: Cython declarations for Antenna
"""

include "config.pxd"
include "eigen.pxd"

# from libcpp.vector cimport vector



cdef extern from "antenna.h":
    ctypedef Vector3f Position

    ctypedef struct Antenna:
        MatrixXf points 
        int id 

    VectorXf compute_delays(const Antenna& antenna)

    Vector3f find_middle(const Antenna& antenna)

    void place_antenna(Antenna& antenna, const Position position)

    Antenna create_antenna(const Position& position,
                        const int columns,
                        const int rows,
                        const float distance)

    Antenna steer(const Antenna& antenna, const float phi, const float theta)

    VectorXf steering_vector(const Antenna& antenna, const Position point)

    MatrixXf generate_unit_dome(const int n)
    # # vector[int] used_sensors(const MatrixXf& antenna, const float distance=DISTANCE)
    # MatrixXf steer(const MatrixXf& antenna, const float azimuth, const float elevation)
    # MatrixXf compute_delays_lookup(const MatrixXf& antenna,
    #                            const int azimuth_resolution,
    #                            const int elevation_resolution,
    #                            const float azimuth_max_angle,
    #                            const float elevation_max_angle)
