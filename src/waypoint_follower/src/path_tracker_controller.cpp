/**
 * @file path_tracker_controller.cpp
 * @brief Path tracking control using PID
 * @author Autonomous Vehicle Team
 * @date December 2025
 * @version 2.0.1 - FIXED
 */

#include "waypoint_follower/path_tracker_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <algorithm>

namespace waypoint_follower {

PathTrackerController::PathTrackerController(
    double dt, double max_steering, double max_linear_vel)
    : dt_(dt), 
      max_steering_angle_(max_steering),
      max_linear_velocity_(max_linear_vel), 
      use_lqr_(false),
      steering_integral_(0.0),
      steering_prev_error_(0.0),
      velocity_integral_(0.0),
      velocity_prev_error_(0.0),
      steering_kp_(1.0),
      steering_ki_(0.1),
      steering_kd_(0.5),
      velocity_kp_(0.5),
      velocity_ki_(0.05),
      velocity_kd_(0.1) {
    
    RCLCPP_INFO(rclcpp::get_logger("PathTrackerController"),
               "Path tracking controller initialized");
}

ControlCommand PathTrackerController::update(
    const RobotState&, const TrackingError& error) {
    
    ControlCommand cmd;
    
    // PID Steering control
    steering_integral_ += error.cross_track_error * dt_;
    double steering_derivative = (error.cross_track_error - steering_prev_error_) / dt_;
    
    cmd.angular_velocity = steering_kp_ * error.cross_track_error +
                          steering_ki_ * steering_integral_ +
                          steering_kd_ * steering_derivative;
    
    steering_prev_error_ = error.cross_track_error;
    
    // PID Velocity control
    velocity_integral_ += error.velocity_error * dt_;
    double velocity_derivative = (error.velocity_error - velocity_prev_error_) / dt_;
    
    cmd.linear_velocity = velocity_kp_ * error.velocity_error +
                         velocity_ki_ * velocity_integral_ +
                         velocity_kd_ * velocity_derivative;
    
    velocity_prev_error_ = error.velocity_error;
    
    // Enforce constraints
    cmd.linear_velocity = std::max(0.0,
        std::min(cmd.linear_velocity, max_linear_velocity_));
    
    cmd.angular_velocity = std::max(-max_steering_angle_,
        std::min(cmd.angular_velocity, max_steering_angle_));
    
    return cmd;
}

void PathTrackerController::set_steering_pid_gains(
    double kp, double ki, double kd) {
    
    steering_kp_ = kp;
    steering_ki_ = ki;
    steering_kd_ = kd;
    
    RCLCPP_INFO(rclcpp::get_logger("PathTrackerController"),
               "Steering PID gains updated: Kp=%.3f, Ki=%.3f, Kd=%.3f",
               kp, ki, kd);
}

void PathTrackerController::set_velocity_pid_gains(
    double kp, double ki, double kd) {
    
    velocity_kp_ = kp;
    velocity_ki_ = ki;
    velocity_kd_ = kd;
    
    RCLCPP_INFO(rclcpp::get_logger("PathTrackerController"),
               "Velocity PID gains updated: Kp=%.3f, Ki=%.3f, Kd=%.3f",
               kp, ki, kd);
}

void PathTrackerController::reset() {
    steering_integral_ = 0.0;
    steering_prev_error_ = 0.0;
    velocity_integral_ = 0.0;
    velocity_prev_error_ = 0.0;
}

double PathTrackerController::compute_lookahead_distance(double velocity) {
    double base_distance = velocity * 2.0;
    return std::max(0.2, base_distance);
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

} // namespace waypoint_follower
