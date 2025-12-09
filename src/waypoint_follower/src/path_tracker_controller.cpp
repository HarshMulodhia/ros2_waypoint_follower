#include "waypoint_follower/path_tracker_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace waypoint_follower {

PathTrackerController::PathTrackerController(
    double dt, double max_steering, double max_linear_vel)
    : dt_(dt), max_steering_angle_(max_steering),
      max_linear_velocity_(max_linear_vel), use_lqr_(false) {
    
    steering_pid_ = std::make_unique<robotics_control::PIDController>(
        1.0, 0.1, 0.5, dt, -max_steering, max_steering);
    velocity_pid_ = std::make_unique<robotics_control::PIDController>(
        0.5, 0.05, 0.1, dt, 0.0, max_linear_vel);
}

ControlCommand PathTrackerController::update(
    const RobotState& state, const TrackingError& error) {
    
    ControlCommand cmd;
    // PIDController expects (setpoint, measurement), where the error itself is the measurement
    // Setpoint is 0 for error-based control (we want error to be zero)
    cmd.angular_velocity = steering_pid_->update(0.0, error.cross_track_error);
    cmd.linear_velocity = velocity_pid_->update(0.0, error.velocity_error);
    
    cmd.linear_velocity = std::max(0.0,
        std::min(cmd.linear_velocity, max_linear_velocity_));
    cmd.angular_velocity = std::max(-max_steering_angle_,
        std::min(cmd.angular_velocity, max_steering_angle_));
    
    return cmd;
}

void PathTrackerController::set_steering_pid_gains(
    double kp, double ki, double kd) {
    steering_pid_ = std::make_unique<robotics_control::PIDController>(
        kp, ki, kd, dt_, -max_steering_angle_, max_steering_angle_);
}

void PathTrackerController::set_velocity_pid_gains(
    double kp, double ki, double kd) {
    velocity_pid_ = std::make_unique<robotics_control::PIDController>(
        kp, ki, kd, dt_, 0.0, max_linear_velocity_);
}

void PathTrackerController::reset() {
    steering_pid_->reset();
    velocity_pid_->reset();
}

double PathTrackerController::compute_lookahead_distance(double velocity) {
    return std::max(0.2, velocity * 2.0);
}

Eigen::Vector2d PathTrackerController::compute_lookahead_point(
    const RobotState& state, const Trajectory& trajectory) {
    
    if (trajectory.empty()) {
        return Eigen::Vector2d(state.x, state.y);
    }
    
    size_t nearest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
        double dist = std::hypot(
            trajectory.points[i].x - state.x,
            trajectory.points[i].y - state.y);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
    }
    
    return Eigen::Vector2d(
        trajectory.points[nearest_idx].x,
        trajectory.points[nearest_idx].y);
}

}  // namespace waypoint_follower
