#include "waypoint_follower/path_tracker_controller.hpp"
#include "control/pid_controller.hpp"
#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

namespace waypoint_follower {

PathTrackerController::PathTrackerController(double dt, double max_steering,
                                             double max_linear_vel)
    : dt_(dt), max_steering_angle_(max_steering),
      max_linear_velocity_(max_linear_vel), use_lqr_(false) {

  // Initialize PID controllers for steering and velocity
  // Steering PID: controls angular velocity based on cross-track error
  steering_pid_ = std::make_unique<robotics_control::PIDController>(
      1.0,                  // Kp - proportional gain
      0.1,                  // Ki - integral gain
      0.5,                  // Kd - derivative gain
      dt,                   // dt - time step
      -max_steering_angle_, // output_min
      max_steering_angle_   // output_max
  );

  // Velocity PID: controls linear velocity based on velocity error
  velocity_pid_ = std::make_unique<robotics_control::PIDController>(
      0.5,                 // Kp - proportional gain
      0.05,                // Ki - integral gain
      0.1,                 // Kd - derivative gain
      dt,                  // dt - time step
      0.0,                 // output_min
      max_linear_velocity_ // output_max
  );

  RCLCPP_INFO(rclcpp::get_logger("PathTrackerController"),
              "Path tracking controller initialized with dt=%.3f, "
              "max_steering=%.3f rad, max_velocity=%.3f m/s",
              dt, max_steering, max_linear_vel);
}

ControlCommand PathTrackerController::update(const RobotState &state,
                                             const TrackingError &error) {
  ControlCommand cmd;
  cmd.linear_velocity = 0.0;
  cmd.lateral_velocity = 0.0;
  cmd.angular_velocity = 0.0;

  // Use velocity from state for lookahead computation
  double lookahead_dist = compute_lookahead_distance(state.vx);

  // Use steering PID for cross-track error
  cmd.angular_velocity =
      steering_pid_->update(0.0,                     // setpoint (zero error)
                            -error.cross_track_error // measurement
      );

  // Use velocity PID
  cmd.linear_velocity = velocity_pid_->update(error.velocity_error, 0.0);

  // Apply heading correction
  double heading_correction = 0.5 * error.heading_error;
  cmd.angular_velocity += heading_correction;

  // Enforce constraints
  cmd.linear_velocity =
      std::max(0.0, std::min(cmd.linear_velocity, max_linear_velocity_));
  cmd.angular_velocity =
      std::max(-max_steering_angle_,
               std::min(cmd.angular_velocity, max_steering_angle_));

  return cmd;
}

void PathTrackerController::set_steering_pid_gains(double kp, double ki,
                                                   double kd) {
  steering_pid_->set_gains(kp, ki, kd);
  RCLCPP_INFO(rclcpp::get_logger("PathTrackerController"),
              "Steering PID gains updated: Kp=%.3f, Ki=%.3f, Kd=%.3f", kp, ki,
              kd);
}

void PathTrackerController::set_velocity_pid_gains(double kp, double ki,
                                                   double kd) {
  velocity_pid_->set_gains(kp, ki, kd);
  RCLCPP_INFO(rclcpp::get_logger("PathTrackerController"),
              "Velocity PID gains updated: Kp=%.3f, Ki=%.3f, Kd=%.3f", kp, ki,
              kd);
}

void PathTrackerController::reset() {
  steering_pid_->reset();
  velocity_pid_->reset();
}

double PathTrackerController::compute_lookahead_distance(double velocity) {
  double base_distance = velocity * 2.0;
  return std::max(0.2, base_distance);
}

Eigen::Vector2d
PathTrackerController::compute_lookahead_point(const RobotState &state,
                                               const Trajectory &trajectory) {

  if (trajectory.empty()) {
    return Eigen::Vector2d(state.x, state.y);
  }

  // Find nearest point on trajectory
  size_t nearest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    double dist = std::hypot(trajectory.points[i].x - state.x,
                             trajectory.points[i].y - state.y);

    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = i;
    }
  }

  // Look ahead by a certain distance
  double lookahead_dist = compute_lookahead_distance(state.vx);

  // Find point that is lookahead_dist away from nearest point
  double accumulated_dist = 0.0;
  for (size_t i = nearest_idx; i < trajectory.points.size() - 1; ++i) {
    double segment_dist =
        std::hypot(trajectory.points[i + 1].x - trajectory.points[i].x,
                   trajectory.points[i + 1].y - trajectory.points[i].y);

    if (accumulated_dist + segment_dist >= lookahead_dist) {
      // Interpolate between i and i+1
      double ratio = (lookahead_dist - accumulated_dist) / segment_dist;
      return Eigen::Vector2d(
          trajectory.points[i].x +
              ratio * (trajectory.points[i + 1].x - trajectory.points[i].x),
          trajectory.points[i].y +
              ratio * (trajectory.points[i + 1].y - trajectory.points[i].y));
    }

    accumulated_dist += segment_dist;
  }

  // Return last point if we can't look ahead enough
  return Eigen::Vector2d(trajectory.points.back().x,
                         trajectory.points.back().y);
}

} // namespace waypoint_follower
