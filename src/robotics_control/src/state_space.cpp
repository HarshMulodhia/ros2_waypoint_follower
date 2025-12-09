#include "control/state_space.hpp"

using namespace Eigen;
namespace robotics_control {

StateSpace::StateSpace(const MatrixXd& A, const MatrixXd& B,
const MatrixXd& C, const MatrixXd& D)
:A_(A), B_(B), C_(C), D_(D) {
    //Set state to zero
    state_ = VectorXd::Zero(A.rows());
}

VectorXd StateSpace::step(const VectorXd& u) {
    //x[k+1]=A*x[k] + B*u[k]
    VectorXd next_state = A_ * state_ + B_ * u;
    //y[k]=C*x[k] + D*u[k]
    VectorXd output = C_ * state_ + D_*u;

    state_=next_state;
    return output;
}

void StateSpace::reset() {
    state_ = VectorXd::Zero(state_.rows());
}

}