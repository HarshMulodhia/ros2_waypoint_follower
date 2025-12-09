#include "estimation/kalman_filter.hpp"

using namespace Eigen;
namespace robotics_control {

KalmanFilter::KalmanFilter(const MatrixXd& A, const MatrixXd& B, 
                          const MatrixXd& C, const MatrixXd& Q, 
                          const MatrixXd& R, const MatrixXd& P0, 
                          const VectorXd& x0)
    : A_(A), B_(B), C_(C), Q_(Q), R_(R), x_hat_(x0), P_(P0) {
    I_ = MatrixXd::Identity(A.rows(), A.rows());
}

void KalmanFilter::prediction_update(const VectorXd& u) {
    // x_hat[k|k-1] = A*x_hat[k-1|k-1] + B*u[k-1]
    x_hat_ = A_ * x_hat_ + B_ * u;
    
    // P[k|k-1] = A*P[k-1|k-1]*A' + Q
    P_ = A_ * P_ * A_.transpose() + Q_;
}

void KalmanFilter::measurement_update(const VectorXd& z) {
    // Innovation: y = z - C*x_hat
    VectorXd y = z - C_ * x_hat_;
    
    // Innovation covariance: S = C*P*C' + R
    MatrixXd S = C_ * P_ * C_.transpose() + R_;
    
    // Kalman gain: K = P*C' / S
    K_ = P_ * C_.transpose() * S.inverse();
    
    // Update state: x_hat = x_hat + K*y
    x_hat_ = x_hat_ + K_ * y;
    
    // Update covariance: P = (I - K*C)*P
    P_ = (I_ - K_ * C_) * P_;
}

void KalmanFilter::reset(const VectorXd& x0, const MatrixXd& P0) {
    x_hat_ = x0;
    P_ = P0;
}

}
