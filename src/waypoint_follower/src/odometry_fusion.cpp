#include "waypoint_follower/odometry_fusion.hpp"
#include "estimation/kalman_filter.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

namespace waypoint_follower {

OdometryFusion::OdometryFusion(double dt) : dt_(dt) {
  build_system_matrices();

  // Initialize Kalman filter with 6-state system
  // State: [x, y, theta, vx, vy, omega]
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(6);
  kalman_filter_ = std::make_unique<robotics_control::KalmanFilter>(
      A_, B_, C_, Q_, R_, P0_, x0);
}

void OdometryFusion::initialize(const RobotState &initial_state) {
  fused_state_ = initial_state;
  Eigen::VectorXd x0(6);
  x0 << initial_state.x, initial_state.y, initial_state.theta, initial_state.vx,
      initial_state.vy, initial_state.omega;

  kalman_filter_->reset(x0, P0_);

  RCLCPP_INFO(rclcpp::get_logger("OdometryFusion"),
              "Kalman filter initialized with state: [%.3f, %.3f, %.3f, %.3f, "
              "%.3f, %.3f]",
              initial_state.x, initial_state.y, initial_state.theta,
              initial_state.vx, initial_state.vy, initial_state.omega);
}

void OdometryFusion::predict_from_odometry(double linear_vel,
                                           double angular_vel) {
  // Update state using kinematic model
  fused_state_.vx = linear_vel;
  fused_state_.omega = angular_vel;

  double cos_theta = std::cos(fused_state_.theta);
  double sin_theta = std::sin(fused_state_.theta);

  fused_state_.x += linear_vel * cos_theta * dt_;
  fused_state_.y += linear_vel * sin_theta * dt_;
  fused_state_.theta += angular_vel * dt_;

  // Normalize theta to [-pi, pi]
  while (fused_state_.theta > M_PI)
    fused_state_.theta -= 2 * M_PI;
  while (fused_state_.theta < -M_PI)
    fused_state_.theta += 2 * M_PI;

  // Prepare input vector for Kalman filter prediction
  Eigen::VectorXd u(3);
  u << linear_vel, angular_vel, 0.0;

  // Perform Kalman prediction step
  kalman_filter_->prediction_update(u);
}

void OdometryFusion::update_from_imu(double accel_x, double accel_y,
                                     double gyro_z) {
  // Measurement from IMU: [gyro_z, accel_x, accel_y]
  Eigen::VectorXd z(3);
  z << gyro_z, accel_x, accel_y;

  // Perform Kalman measurement update
  kalman_filter_->measurement_update(z);

  // Extract updated state from Kalman filter
  const Eigen::VectorXd &x_hat = kalman_filter_->get_state();
  fused_state_.x = x_hat(0); // ADD THIS LINE
  fused_state_.y = x_hat(1);
  fused_state_.theta = x_hat(2);
  fused_state_.vx = x_hat(3);
  fused_state_.vy = x_hat(4);
  fused_state_.omega = x_hat(5);

  RCLCPP_DEBUG(rclcpp::get_logger("OdometryFusion"),
               "IMU update: ax=%.3f, ay=%.3f, gyro=%.3f", accel_x, accel_y,
               gyro_z);
}

void OdometryFusion::update_from_encoder(double left_ticks, double right_ticks,
                                         double wheel_base,
                                         double wheel_radius) {
  // Convert wheel encoder ticks to velocities
  double left_velocity = left_ticks * wheel_radius / dt_;
  double right_velocity = right_ticks * wheel_radius / dt_;

  // Differential drive kinematics
  double linear_vel = (left_velocity + right_velocity) / 2.0;
  double angular_vel = (right_velocity - left_velocity) / wheel_base;

  // Use existing predict function
  predict_from_odometry(linear_vel, angular_vel);
}

void OdometryFusion::set_process_noise(double q_val) { Q_ = build_Q(q_val); }

void OdometryFusion::set_measurement_noise(double r_val) {
  R_ = build_R(r_val);
}

void OdometryFusion::reset(const RobotState &initial_state) {
  initialize(initial_state);
}

const RobotState &OdometryFusion::get_fused_state() const {
  return fused_state_;
}

void OdometryFusion::build_system_matrices() {
  // State transition matrix A (6x6)
  A_ = build_A();

  // Process noise covariance Q (6x6)
  Q_ = build_Q(0.01);

  // Measurement matrix C (3x6)
  // Maps state to measurements: [theta, accel_x, accel_y]
  C_ = build_C();

  // Measurement noise covariance R (3x3)
  R_ = build_R(0.1);

  // Initial state covariance P0 (6x6)
  P0_ = Eigen::MatrixXd::Identity(6, 6) * 0.1;
}

Eigen::MatrixXd OdometryFusion::build_A() {
  // State transition matrix for kinematic model
  // x[k+1] = A*x[k] + B*u[k]
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6, 6);
  A(0, 3) = dt_; // x += vx * dt
  A(1, 4) = dt_; // y += vy * dt
  A(2, 5) = dt_; // theta += omega * dt
  return A;
}

Eigen::MatrixXd OdometryFusion::build_B() {
  // Input matrix B (6x3)
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6, 3);
  B(3, 0) = 1.0; // vx input
  B(4, 1) = 1.0; // vy input
  B(5, 2) = 1.0; // omega input
  return B;
}

Eigen::MatrixXd OdometryFusion::build_Q(double q_val) {
  // Process noise covariance (6x6)
  return Eigen::MatrixXd::Identity(6, 6) * q_val;
}

Eigen::MatrixXd OdometryFusion::build_C() {
  // Measurement matrix C (3x6)
  // Measures: [gyro_z (theta rate), accel_x, accel_y]
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3, 6);
  C(0, 5) = 1.0; // Measure omega from state
  C(1, 3) = 1.0; // Measure vx from state
  C(2, 4) = 1.0; // Measure vy from state
  return C;
}

Eigen::MatrixXd OdometryFusion::build_R(double r_val) {
  // Measurement noise covariance (3x3)
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3) * r_val;
  R(0, 0) = 0.01; // Gyro noise (low - gyros are accurate)
  R(1, 1) = 0.1;  // Accel X noise
  R(2, 2) = 0.1;  // Accel Y noise
  return R;
}

} // namespace waypoint_follower
