#pragma once

#include <eigen3/Eigen/Dense>

namespace robotics_control {
/**
 * @class StateSpace
 * @brief Discrete-time linear state-space model
 * 
 * Implements the discrete-time system:
 * x[k+1] = A*x[k] + B*u[k]
 * y[k] = C*x[k] + D*u[k]
 */
    
class StateSpace
{
    
private:
    Eigen::MatrixXd A_, B_, C_, D_;
    Eigen::VectorXd state_;
public:
    /**
     * @brief Constructor
     * @param A State transition matrix (nxn)
     * @param B Input matrix (nxm)
     * @param C Output matrix (pxn)
     * @param D Feedthrough matrix (pxm)
     */
    StateSpace(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& C, const Eigen::MatrixXd& D);

    /**
     * @brief Step the system forward in time
     * @param u Control input
     * @return Output y[k]
     */
    Eigen::VectorXd step(const Eigen::VectorXd& u);

    /**
     * @brief Reset state to zero
     */
    void reset();

    /**
     * @brief Set State
     */
    void set_state(const Eigen::VectorXd& x) {state_ = x;}

    /**
     * @brief Get current state
     */
    Eigen::VectorXd get_state() const {return state_;}

};

}