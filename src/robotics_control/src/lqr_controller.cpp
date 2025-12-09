#include "control/lqr_controller.hpp"
#include <eigen3/Eigen/Dense>

using namespace Eigen;
namespace robotics_control {

LQRController::LQRController(const MatrixXd& K) : K_(K) {}

VectorXd LQRController::compute_input(const VectorXd& state) {
    return -K_ * state; // u = -K*x
}

}
