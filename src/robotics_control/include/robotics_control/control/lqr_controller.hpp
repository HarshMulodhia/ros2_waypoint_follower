#pragma once

#include <eigen3/Eigen/Dense>

namespace robotics_control {

/**
 * @class LQRController
 * @brief Linear Quadratic Regulator (LQR) controller for state feedback
 * 
 * Minimizes cost: J = sum(x'*Q*x + u'*R*u)
 * with feedback law: u = -K*x
 */
class LQRController {
    using MatrixXd = Eigen::MatrixXd;
    using VectorXd = Eigen::VectorXd;

private:
    MatrixXd K_;  // Feedback gain matrix
public:
    /**
     * @brief Constructor with precomputed gain matrix
     * @param K Feedback gain matrix (m x n)
     */
    LQRController(const MatrixXd& K);

    /**
     * @brief Compute control input
     * @param state Current state vector
     * @return Control input u = -K*x
     */
    VectorXd compute_input(const VectorXd& state);

    /**
     * @brief Set gain matrix
     */
    void set_gain(const MatrixXd& K) { K_ = K; }

    /**
     * @brief Get gain matrix
     */
    MatrixXd get_gain() const { return K_; }

};

} 