# cython: language_level=3
# distutils: language=c++

"""
This module contains interface for the antenna 
"""

"""# distutils: sources = matrix_mult.cpp"""


__author__ = "Irreq"
__copyright__ = "Copyright Info"
__credits__ = ["Irreq"]
__license__ = "License Name and Info"
__version__ = "0.1.0"
__maintainer__ = "Irreq"
__email__ = ""
__status__ = "development"

import numpy as np
cimport numpy as np

# It's necessary to call "import_array" if you use any part of the numpy PyArray_* API.
np.import_array()


include "config.pxd"
include "antenna.pxd"

cdef VectorXfToNumpy(VectorXf vec):
    result = np.zeros(vec.rows(), dtype=np.float32)
    for i in range(vec.rows()):
        result[i] = vec.coeff(i)
    return result

cdef MatrixXfToNumpy(MatrixXf matrix):
    """
    Convert a Eigen.MatrixXf to numpy.ndarray
    """
    result = np.zeros((matrix.rows(),matrix.cols())) # create nd array
    # Fill out the nd array with MatrixXf elements 
    for row in range(matrix.rows()): 
        for col in range(matrix.cols()): 
            result[row, col] = matrix.coeff(row, col)

    return result

# def create_combined_array(definition: list[list[int]], position: np.ndarray[1] = ORIGO):
#     """Create a 2D array superstructure of smaller arrays
#
#     Args:
#         definition (list[list[int]]): How the arrays should be positioned
#         position (np.ndarray[1], optional): Where the arrays will be located. Defaults to ORIGO.
#
#     Returns:
#         _type_: The new array
#     """
#     antennas = []
#
#     SEPARATION_X = COLUMNS * DISTANCE + ARRAY_SEPARATION / 2
#     SEPARATION_Y = ROWS * DISTANCE + ARRAY_SEPARATION / 2
#
#     for i in range(len(definition)):
#         for j in range(len(definition[i])):
#             if definition[i][j]:
#                 antenna = create_antenna(
#                     np.array([SEPARATION_X * j, SEPARATION_Y * i, 0])
#                 )
#                 antennas.append(antenna)
#
#     # Create a final big array
#     combined_array = np.concatenate(antennas)
#
#     return place_antenna(combined_array, position)

def dome(n: int) -> np.ndarray:
    result = MatrixXfToNumpy(generate_unit_dome(n))
    return result


def generate_angles_sphere(n: int):
    """Create n angles on a unitsphere

    Args:
        n (int): number of points to create

    Returns:
        (float, float): polar angles to each point
    """
    i = np.arange(0, n, dtype=np.float32) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    golden_ratio = (1 + 5**0.5) / 2

    theta = 2 * np.pi * i / golden_ratio

    return theta, phi


def generate_points_half_sphere(n: int):
    """Create n points on a dome

    Args:
        n (int): number of points

    Returns:
        np.ndarray: points -> (n, 3)
    """

    theta, phi = generate_angles_sphere(2 * n)

    valid = (np.pi / 2) >= phi

    theta = theta[valid]
    phi = phi[valid]

    x = np.cos(theta) * np.sin(phi)
    y = np.sin(theta) * np.sin(phi)
    z = np.cos(phi)

    points = np.vstack((x, y, z))

    return points.T


def convert_points_to_polar_angles(points: np.ndarray):
    """Converts points to polar angles

    Args:
        points (np.ndarray): the points that will be converted

    Returns:
        (np.ndarray): polar angles -> (n, 2)
    """
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    r = np.sqrt(x**2 + y**2 + z**2)
    phi = np.arccos(z / r)
    theta = np.arctan2(y, x)
    theta = np.arccos(z / r)
    phi = np.arctan2(y, x)
    return np.vstack((theta, phi)).T


def convert_points_to_steering_angles(points: np.ndarray):
    """Converts points to steering angles

    Args:
        points (np.ndarray): the points that will be converted

    Returns:
        (np.ndarray): steering angles -> (n, 2)
    """
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    azimuth = np.degrees(np.arctan2(z, y) - np.pi / 2)
    elevation = np.degrees(np.arctan2(z, x) - np.pi / 2)

    # azimuth = np.degrees(np.arctan2(z, x) - np.pi / 2)
    # elevation = np.degrees(np.arctan2(z, np.sqrt(x**2 + y**2)))

    return np.vstack((azimuth, elevation)).T



# cdef ok():
#     cdef Position pos = Vector3f(0.0, 0.0, 0.0)
#
#     cdef Antenna antenna = create_antenna(pos, COLUMNS, ROWS, DISTANCE)
#
#     cdef MatrixXf steered = steer(antenna, 10.0, 5.0)
#
#
#
#     # me = antenna
#     # result = np.zeros((me.rows(),me.cols())) # create nd array
#     # print(result.shape)
#     # # Fill out the nd array with MatrixXf elements 
#     # for row in range(me.rows()): 
#     #     for col in range(me.cols()): 
#     #         result[row, col] = me.coeff(row, col)
#
#     cdef VectorXf delays = compute_delays(steered)
#
#     result = np.zeros((ROWS, COLUMNS), dtype=np.float32)
#
#     i = 0
#     for y in range(ROWS):
#         for x in range(COLUMNS):
#             result[y, x] = delays.coeff(i)
#             i += 1
#
#
#
#     return result
#
# if i have a cpp class known as A that i have imported to cython.

# How can i call a cython function with that class? that is:

# cdef bar(A a):
#     a.do_domething() # This does not work gives: Cannot convert Python object argument to type 'A'

# cdef foo(): 
#     cdef A a = A()

#     a.do_domething() # This works

# ctypedef np.ndarray[np.float32_t, ndim=1, order="c"]

# cimport eigen

# np.ndarray[np.float32_t, ndim=1, mode="c"]

#cdef np.ndarray[np.float32_t, ndim=2, mode="c"] PyAntenna 
#PyPosition = np.ndarray[np.float32_t, ndim=1, mode="c"] 

ORIGO: np.ndarray = np.zeros(3, dtype=np.float32)

cdef class Array:

    """
    Module interface for antenna 
    """

    cdef Antenna antenna, steered
    cdef Position pos

    # cdef Antenna steered

    cdef object antenna_repr

    def __init__(self, int rows, int cols, float distance=DISTANCE, position = ORIGO):
        self.pos = Vector3f(position[0], position[1], position[2])
        self.antenna = create_antenna(self.pos, cols, rows, distance)
        self.toNumpy()

    cdef public np.ndarray toNumpy(self):
        self.antenna_repr = np.zeros((self.antenna.points.rows(),
                                      self.antenna.points.cols()), dtype=np.float32)

        for row in range(self.antenna.points.rows()):
            for col in range(self.antenna.points.cols()):
                self.antenna_repr[row, col] = self.antenna.points.coeff(row, col)

        return self.antenna_repr

    def __getitem__(self, int idx):
        if not (-self.antenna.points.rows() < idx < self.antenna.points.rows()):
            raise IndexError("Out of bounds error")

        return self.antenna_repr[idx]

    def steer(self, float azimuth, float elevation):
        self.steered = steer(self.antenna, azimuth, elevation)

        return MatrixXfToNumpy(self.steered.points)
    #
    # def get_delay_vector(self):
    #     return VectorXfToNumpy(compute_delays(self.steered))

    def _toNumpy(self):
        return self.toNumpy()




# cdef object bar(MatrixXf antenna):
#     print(antenna.cols(), 276347263476327)
#
#     return np.zeros(5)


# cpdef np.ndarray toNumpy(MatrixXf& antenna):
#     result = np.zeros((antenna.rows(), antenna.cols()), dtype=np.float32)

#     for row in range(antenna.rows()):
#         for col in range(antenna.cols()):
#             result[row, col] = antenna.coeff(row, col)

#     return result

# Antenna creation

cdef np.ndarray[np.float32_t, ndim=2, mode="c"] cy_create_antenna(np.ndarray position):
    cdef Position pos = Vector3f(position[0], position[1], position[2])
    cdef Antenna antenna = create_antenna(pos, COLUMNS, ROWS, DISTANCE)

    # return toNumpy(antenna)

    # print(bar(antenna))

    result = np.zeros((antenna.points.rows(), antenna.points.cols()), dtype=np.float32)

    for row in range(antenna.points.rows()):
        for col in range(antenna.points.cols()):
            result[row, col] = antenna.points.coeff(row, col)

    return result

def _create_antenna(position: np.ndarray) -> np.ndarray:
    """Wrapper for cy_create_antenna"""
    return cy_create_antenna(position)


# def test():
#     #a = Array(2, 2)
#     # print(a.toNumpy(), 696969, a[2])
#     # cdef Eigen.MatrixXf a 
#     # print(arr())
#     antenna = ok()
#     # print(antenna)
#     # print("Tested Eigen array")
#     return antenna
