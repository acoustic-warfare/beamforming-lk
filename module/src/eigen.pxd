# cython: language_level=3
# distutils: language=c++

"""
File: eigen.pxd 
Author: Irreq 
Date: 2023-10-20

Description: Cython declarations for Eigen
"""

cdef extern from "Eigen/Dense" namespace "Eigen":
    cdef cppclass MatrixXf:
        MatrixXf() except +

        MatrixXf(int d1, int d2) except +
        MatrixXf(MatrixXf other) except +
        void setOnes()
        void setZero()
        int rows()
        int cols()
        float coeff(int, int)

        float *data()
        float& element "operator()"(int row, int col)

    cdef cppclass VectorXf:
        VectorXf() except +
        void setOnes()
        void setZero()
        int rows()
        int cols()
        float coeff(int)
    cdef cppclass Vector3f:
        Vector3f() except +
        Vector3f(float a, float b, float c)
