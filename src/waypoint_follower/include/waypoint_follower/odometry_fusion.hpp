/**
 * @file odometry_fusion.hpp
 * @brief Multi-sensor state estimation via Kalman filtering
 * @author Harsh Mulodhia
 * @version 1.0.0
 *
 * @class OdometryFusion
 * @brief Extended Kalman Filter for 6-DOF robot state estimation
 *
 * Fuses multiple sensor inputs:
 * - **Odometry:** Wheel encoder data → velocity estimates
 * - **IMU:** Accelerometer + gyroscope → velocity and rate updates
 * - **Kinematics:** Motion model → state prediction
 *
 * State representation:
 * @code
 * x = [x, y, θ, vx, vy, ω]^T
 * - (x, y): Position in world frame
 * - θ: Heading angle
 * - (vx, vy): Linear velocities
 * - ω: Angular velocity
 * @endcode
 *
 * Filter stages:
 * 1. **Prediction:** Apply kinematic model + process noise
 * 2. **Measurement Update:** Fuse sensor data with measurement noise
 *
 * Tuning parameters:
 * - process_noise_q: Trust in motion model (lower = more drift correction)
 * - measurement_noise_r: Trust in sensors (lower = more responsive)
 *
 * @note Integrates robotics_control::KalmanFilter for filter implementation
 * @see initialize(), predict_from_odometry(), update_from_imu()
 */


#pragma once

#include <Eigen/Dense>
#include <memory>

#include "estimation/kalman_filter.hpp"
#include "waypoint_follower/waypoint_types.hpp"

namespace waypoint_follower {

class OdometryFusion {
private:
  double dt_;
  RobotState fused_state_;

  // Kalman filter for state estimation
  std::unique_ptr<robotics_control::KalmanFilter> kalman_filter_;

  // System matrices
  Eigen::MatrixXd A_, B_, C_, Q_, R_, P0_;

  // Helper methods
  void build_system_matrices();
  Eigen::MatrixXd build_A();
  Eigen::MatrixXd build_B();
  Eigen::MatrixXd build_Q(double q_val);
  Eigen::MatrixXd build_C();
  Eigen::MatrixXd build_R(double r_val);

public:

  /**
   * Initialize odometry fusion with Kalman filter
   * @param dt Sampling time (seconds), must match control loop rate
   */
  explicit OdometryFusion(double dt);

  /**
   * Initialize filter with known initial state
   * @param initial_state Starting robot state
   */
  void initialize(const RobotState &initial_state);

  /**
   * Prediction step: update state using kinematic model
   * @param linear_vel Forward velocity command (m/s)
   * @param angular_vel Rotational velocity command (rad/s)
   * 
   * Kinematic Update:
   *   x[k+1] = x[k] + v_x * cos(θ) * dt
   *   y[k+1] = y[k] + v_x * sin(θ) * dt
   *   θ[k+1] = θ[k] + ω * dt
   *   (normalize θ to [-π, π])
   */
  void predict_from_odometry(double linear_vel, double angular_vel);

  /**
   * Measurement update: incorporate IMU accelerations and gyro
   * @param accel_x Acceleration in X (m/s²)
   * @param accel_y Acceleration in Y (m/s²)
   * @param gyro_z Angular velocity from gyroscope (rad/s)
   * 
   * Measurement Vector: z = [gyro_z, accel_x, accel_y]^T
   */
  void update_from_imu(double accel_x, double accel_y, double gyro_z);

  /**
   * Measurement update: use wheel encoder data
   * @param left_ticks Encoder ticks from left wheel
   * @param right_ticks Encoder ticks from right wheel
   * @param wheel_base Distance between wheels (meters)
   * @param wheel_radius Wheel radius (meters)
   * 
   * Converts encoder ticks to differential drive kinematics:
   *   v_left = left_ticks * wheel_radius / dt
   *   v_right = right_ticks * wheel_radius / dt
   *   v_linear = (v_left + v_right) / 2
   *   ω = (v_right - v_left) / wheel_base
   */
  void update_from_encoder(double left_ticks, double right_ticks,
                           double wheel_base, double wheel_radius);

  /**
   * Configure process noise covariance
   * @param q_val Noise intensity (typical: 0.01 - 0.1)
   * @details Higher values → more aggressive tracking of measurements
   */
  void set_process_noise(double q_val);

  /**
   * Configure measurement noise covariance
   * @param r_val Noise intensity (typical: 0.1 - 1.0)
   * @details Higher values → more trust in predictions vs measurements
   */
  void set_measurement_noise(double r_val);

  /// Reset filter to initial state
  void reset(const RobotState &initial_state);

  /// Get latest fused state estimate
  const RobotState &get_fused_state() const;
};

} // namespace waypoint_follower