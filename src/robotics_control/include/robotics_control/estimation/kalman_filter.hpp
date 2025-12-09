#pragma once

#include<eigen3/Eigen/Dense>

namespace robotics_control {

    /**
     * @class KalmanFilter
     * @brief Discrete-time Kalman Filter for linear systems
     * 
     * State estimate with covariance:
     * x_hat[k|k] := E[x[k]  | measurements up to k]
     * P[k|k] := covariance of estimation error
     * 
     * Prediction: 
     *  x_hat[k|k-1] = A*x_hat[k-1|k-1] + B*u[k-1]
     *  P[k|k-1] = A*P[k-1|k-1]*A' + Q
     * 
     * Measurement Update:
     *  K[k] = P[k|k-1]*C' / (C*P[k|k-1]*C' + R)
     *  x_hat[k|k] = x_hat[k|k-1] + K[k]*(z[k] - c*x[_hat[k|k-1]])
     *  P[k|k] = (I - K[k] *C) * P[k|k-1]
     */
class KalmanFilter {

private:
    Eigen::MatrixXd A_, B_, C_;    //System Matrices
    Eigen::MatrixXd Q_, R_;    //Noise covariances
    Eigen::VectorXd x_hat_;    //State Estimate
    Eigen::MatrixXd P_, K_;    // Estimate covariance and Kalman Gain
    Eigen::MatrixXd I_;    //Identity Matrix
public:
    /**
     * @brief Constructor
     * @param A State transition matrix
     * @param B Input matrix
     * @param C Measurement matrix
     * @param Q Process noise covariance
     * @param R Measurement noise covariance
     * @param P0 Initial state covariance
     * @param x0 Initial state estimate
     */
    KalmanFilter(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& P0, const Eigen::VectorXd& x0);

    /**
     * @brief Prediction update
     * @param u Contol input
     */
    void prediction_update(const Eigen::VectorXd& u);

    /**
     * @brief Measurement update
     * @param z Measurement vector
     */
    void measurement_update(const Eigen::VectorXd& z);

    /**
     * @brief Get state estimate
     */
    Eigen::VectorXd get_state() const {return x_hat_;}

    /**
     * @brief Get convariance, P
     */
    Eigen::MatrixXd get_covariance() const {return P_;}

    /**
     * @brief Get Kalman gain
     */
    Eigen::MatrixXd get_kalman_gain() const { return K_;}

    /**
     * @brief Reset filter
     */
    void reset(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);
};

}