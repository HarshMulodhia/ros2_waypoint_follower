#include "control/pid_controller.hpp"
#include <algorithm>

namespace robotics_control {

PIDController::PIDController(double kp, double ki, double kd, 
    double dt, double output_min, double output_max)
    : kp_(kp), ki_(ki), kd_(kd), dt_(dt),
    output_min_(output_min), output_max_(output_max),
    error_(0.0), integral_(0.0), derivative_(0.0) {}

void PIDController::reset(){
    error_ = 0.0;integral_=0.0;derivative_=0.0;
}

double PIDController::update(double setpoint, double measurement){
    error_ = setpoint - measurement;
    integral_ += error_*dt_; // with anti-windup
    derivative_ = (error_ - prev_error_) / dt_;

    //PID formula
    double output = kp_*error_ + ki_*integral_ + kd_ * derivative_;

    //Saturated output
    output = std::clamp(output, output_min_, output_max_);

    //Anti-windup: limit integral if saturated
    if(output == output_min_ || output == output_max_) {
        integral_ -= error_*dt_; // undo the integral accumulation
    }

    prev_error_ = error_;
    return output;
}

void PIDController::set_gains(double kp, double ki, double kd) {
    kp_ = kp; ki_ = ki; kd_ = kd;
}

}