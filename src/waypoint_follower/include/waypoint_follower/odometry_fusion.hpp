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
  explicit OdometryFusion(double dt);

  void initialize(const RobotState &initial_state);

  void predict_from_odometry(double linear_vel, double angular_vel);

  void update_from_imu(double accel_x, double accel_y, double gyro_z);

  void update_from_encoder(double left_ticks, double right_ticks,
                           double wheel_base, double wheel_radius);

  void set_process_noise(double q_val);

  void set_measurement_noise(double r_val);

  void reset(const RobotState &initial_state);

  const RobotState &get_fused_state() const;
};

} // namespace waypoint_follower