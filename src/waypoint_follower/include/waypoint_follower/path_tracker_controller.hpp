#pragma once

#include <Eigen/Dense>
#include <memory>

#include "control/pid_controller.hpp"
#include "waypoint_follower/waypoint_types.hpp"

namespace waypoint_follower {

class PathTrackerController {
private:
  double dt_;
  double max_steering_angle_;
  double max_linear_velocity_;
  bool use_lqr_;

  // PID controllers for steering and velocity
  std::unique_ptr<robotics_control::PIDController> steering_pid_;
  std::unique_ptr<robotics_control::PIDController> velocity_pid_;

public:
  PathTrackerController(double dt, double max_steering, double max_linear_vel);

  ControlCommand update(const RobotState &state, const TrackingError &error);

  void set_steering_pid_gains(double kp, double ki, double kd);

  void set_velocity_pid_gains(double kp, double ki, double kd);

  void reset();

  double compute_lookahead_distance(double velocity);

  Eigen::Vector2d compute_lookahead_point(const RobotState &state,
                                          const Trajectory &trajectory);
};

} // namespace waypoint_follower