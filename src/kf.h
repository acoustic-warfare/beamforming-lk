/** @file kf.h
 * @author Irreq
 * @brief TODO:
*/

#ifndef KALMAN
#define KALMAN

#include <Eigen/Dense>
#include <cmath>

/*
 * A great guide
 * https://nbviewer.org/github/balzer82/Kalman/blob/master/Kalman-Filter-CA-Ball.ipynb?create=1
 *
 */

/**
 * @class KalmanFIlter3D
 * @brief TODO:
 */
class KalmanFilter3D {
private:
    Eigen::Matrix<float, 9, 9> A;// State transition matrix
    Eigen::Matrix<float, 9, 9> Q;// Process noise covariance
    Eigen::Matrix<float, 3, 9> H;// Measurement matrix
    Eigen::Matrix<float, 3, 3> R;// Measurement noise covariance
    Eigen::Matrix<float, 9, 9> P;// Error covariance matrix
    Eigen::Vector<float, 9> x;   // State vector
    Eigen::Vector<float, 9> B;   // Disturbance
    Eigen::Matrix<float, 9, 9> I;// Identity

public:
    KalmanFilter3D(float dt) {
        float dt2 = pow(dt, 2);
        float dt3 = pow(dt, 3);
        float dt4 = pow(dt, 4);
        float dt5 = pow(dt, 5);
        float dt6 = pow(dt, 6);

        // Dynamic Matrix
        A << 1.0, 0.0, 0.0, dt, 0.0, 0.0, 1.0 / 2.0 * dt2, 0.0, 0.0,   //
                0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 1.0 / 2.0 * dt2, 0.0,//
                0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 1.0 / 2.0 * dt2,//
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0,            //
                0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0,            //
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt,            //
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,           //
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,           //
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;           //

        // Measurment Matrix
        H << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   //
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,//
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;//

        float rp = pow(1.0, 2);

        // Measurement Noise Covariance Matrix R
        R << rp, 0.0, 0.0,   //
                0.0, rp, 0.0,//
                0.0, 0.0, rp;//

        float sj = 1.0;

        Q << dt6 / 36, 0.0, 0.0, dt5 / 12, 0.0, 0.0, dt4 / 6, 0.0, 0.0,   //
                0.0, dt6 / 36, 0.0, 0.0, dt5 / 12, 0.0, 0.0, dt4 / 6, 0.0,//
                0.0, 0.0, dt6 / 36, 0.0, 0.0, dt5 / 12, 0.0, 0.0, dt4 / 6,//
                dt5 / 12, 0.0, 0.0, dt4 / 4, 0.0, 0.0, dt3 / 2, 0.0, 0.0, //
                0.0, dt5 / 12, 0.0, 0.0, dt4 / 4, 0.0, 0.0, dt3 / 2, 0.0, //
                0.0, 0.0, dt5 / 12, 0.0, 0.0, dt4 / 4, 0.0, 0.0, dt3 / 2, //
                dt4 / 6, 0.0, 0.0, dt3 / 2, 0.0, 0.0, dt2, 0.0, 0.0,      //
                0.0, dt4 / 6, 0.0, 0.0, dt3 / 2, 0.0, 0.0, dt2, 0.0,      //
                0.0, 0.0, dt4 / 6, 0.0, 0.0, dt3 / 2, 0.0, 0.0, dt2;      //

        Q = Q * pow(sj, 2);

        I.setIdentity();
        B.setZero();

        P.setIdentity();
        x.setZero();
    }

    void update(const Eigen::Vector3f &measurement) {
        // Prediction
        x = A * x;// + B* u
        P = A * P * A.transpose() + Q;

        // Kalman gain calculation
        Eigen::Matrix3f S = H * P * H.transpose() + R;
        Eigen::Matrix<float, 9, 3> K = P * H.transpose() * S.inverse();

        // Update step
        Eigen::Vector3f y = measurement - H * x;
        x = x + K * y;
        P = (I - K * H) * P;
    }

    Eigen::Vector3f getState() const { return x.head(3); }
    Eigen::Vector3f getVelocity() const {
        Eigen::Vector3f res;
        res << x(3), x(4), x(5);//(x.begin() + 3, x.begin() + 6);
        return res;
    }

    Eigen::Vector3f predict(int N) {
        Eigen::Matrix<float, 9, 9> An = A;
        Eigen::Vector<float, 9> xn = x;

        // Apply state transition matrix N times
        for (int i = 0; i < N; ++i) {
            xn = An * xn;
            An = An * A;
        }

        return xn.head(3);
    }

    Eigen::Vector3f predict(float t) {
        if (t > 10) {
            return getState();
        }

        int lower = (int) t;
        int higher = lower + 1;

        Eigen::Matrix<float, 9, 9> An = A;
        Eigen::Vector<float, 9> xn = x;

        Eigen::Vector<float, 9> xp = x;

        // Apply state transition matrix N times
        for (int i = 0; i < higher; ++i) {
            xn = An * xn;
            An = An * A;
            if (i == lower) {
                xp = xn;
            }
        }

        Eigen::Vector3f xp3, xn3;
        xp3 = xp.head(3);
        xn3 = xn.head(3);

        float p = t - (float) lower;
        xp3 = xp3 * p + xn3 * (1.0 - p);
        return xp3;

        // return xp.slerp(t - (float)lower, xn);

        // return xn.head(3);
    }
};

#endif