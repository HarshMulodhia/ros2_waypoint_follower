/**
 * @file path_tracker_controller.hpp
 * @brief Path tracking controller using dual PID loops
 * @author Harsh Mulodhia
 * @version 1.0.0
 *
 * @class PathTrackerController
 * @brief Real-time path tracking using cascaded PID control
 *
 * Implements two independent PID loops:
 * - **Steering Loop:** Cross-track error → angular velocity
 *   - Computes lateral error (perpendicular distance to path)
 *   - Applies PID to generate angular velocity command
 * - **Velocity Loop:** Velocity error → linear velocity
 *   - Computes difference from desired velocity
 *   - Applies PID to generate linear velocity command
 *
 * Additional features:
 * - Heading correction proportional to heading error
 * - Look-ahead distance computation for future tracking
 * - Output saturation to respect robot constraints
 * - Real-time parameter tuning via set_*_pid_gains()
 *
 * Control equations:
 * @code
 * ω = Kp_steering * e_cte + Ki_steering * ∫e_cte + Kd_steering * d(e_cte)/dt
 * v = Kp_velocity * e_vel + Ki_velocity * ∫e_vel + Kd_velocity * d(e_vel)/dt
 * @endcode
 */

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

  /**
   * Initialize path tracker with control parameters
   * @param dt Sampling time (seconds), typical: 0.02s (50 Hz)
   * @param max_steering Maximum steering angle (radians), typical: 0.5 rad
   * @param max_linear_vel Maximum linear velocity (m/s), typical: 1.5 m/s
   */
  PathTrackerController(double dt, double max_steering, double max_linear_vel);

  /**
   * Compute control command from state and error
   * @param state Current robot state from odometry/Kalman filter
   * @param error Tracking error computed by node
   * @return ControlCommand for publishing to /cmd_vel
   * 
   * Algorithm:
   *   1. Steering PID: ω = K_p * e_cte + K_i * ∫e_cte + K_d * d(e_cte)/dt
   *   2. Velocity PID: v = K_p * e_vel + K_i * ∫e_vel + K_d * d(e_vel)/dt
   *   3. Heading correction: ω += α * e_theta (proportional gain)
   *   4. Saturate outputs to max values
   */
  ControlCommand update(const RobotState &state, const TrackingError &error);

  /**
   * Set steering PID gains online
   * @param kp Proportional gain (typical: 1.0)
   * @param ki Integral gain (typical: 0.1)
   * @param kd Derivative gain (typical: 0.5)
   */
  void set_steering_pid_gains(double kp, double ki, double kd);

  /**
   * Set velocity PID gains online
   * @param kp Proportional gain (typical: 0.5)
   * @param ki Integral gain (typical: 0.05)
   * @param kd Derivative gain (typical: 0.1)
   */
  void set_velocity_pid_gains(double kp, double ki, double kd);

  /// Reset PID integrator and state
  void reset();

  /**
   * Compute dynamic look-ahead distance
   * @param velocity Current forward velocity (m/s)
   * @return Look-ahead distance (meters)
   * @details Implements simple velocity-dependent look-ahead:
   *   d_lookahead = velocity * 2.0, min 0.2m, max depends on trajectory length
   */
  double compute_lookahead_distance(double velocity);

  /**
   * Find future target point on trajectory
   * @param state Current robot state
   * @param trajectory Current trajectory
   * @return 2D point on trajectory at look-ahead distance
   * @details Used for alternative pursuit-based tracking (optional enhancement)
   */
  Eigen::Vector2d compute_lookahead_point(const RobotState &state,
                                          const Trajectory &trajectory);
};

} // namespace waypoint_follower