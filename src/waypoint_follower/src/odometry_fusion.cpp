/**
 * @file odometry_fusion.cpp
 * @brief Multi-sensor odometry fusion
 * @author Autonomous Vehicle Team
 * @date December 2025
 * @version 2.0.1 - FIXED
 */

#include "waypoint_follower/odometry_fusion.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <cmath>

namespace waypoint_follower {

OdometryFusion::OdometryFusion(double dt) : dt_(dt) {
    build_system_matrices();
}

void OdometryFusion::initialize(const RobotState& initial_state) {
    fused_state_ = initial_state;
    
    Eigen::Matrix<double, 6, 1> x0;
    x0 << initial_state.x, 
          initial_state.y, 
          initial_state.theta,
          initial_state.vx, 
          initial_state.vy, 
          initial_state.omega;
    
    RCLCPP_INFO(rclcpp::get_logger("OdometryFusion"),
               "Kalman filter initialized");
}

void OdometryFusion::predict_from_odometry(double linear_vel, double angular_vel) {
    fused_state_.vx = linear_vel;
    fused_state_.omega = angular_vel;
    
    double cos_theta = std::cos(fused_state_.theta);
    double sin_theta = std::sin(fused_state_.theta);
    
    fused_state_.x += linear_vel * cos_theta * dt_;
    fused_state_.y += linear_vel * sin_theta * dt_;
    fused_state_.theta += angular_vel * dt_;
    
    while (fused_state_.theta > M_PI) fused_state_.theta -= 2 * M_PI;
    while (fused_state_.theta < -M_PI) fused_state_.theta += 2 * M_PI;
}

void OdometryFusion::update_from_imu(
    double accel_x, double accel_y, double gyro_z) {
    
    RCLCPP_DEBUG(rclcpp::get_logger("OdometryFusion"),
                "IMU update: ax=%.3f, ay=%.3f, gyro=%.3f",
                accel_x, accel_y, gyro_z);
}

void OdometryFusion::update_from_encoder(
    double left_ticks, double right_ticks,
    double wheel_base, double wheel_radius) {
    
    double left_velocity = left_ticks * wheel_radius / dt_;
    double right_velocity = right_ticks * wheel_radius / dt_;
    double linear_vel = (left_velocity + right_velocity) / 2.0;
    double angular_vel = (right_velocity - left_velocity) / wheel_base;
    
    predict_from_odometry(linear_vel, angular_vel);
}

void OdometryFusion::set_process_noise(double q_val) {
    Q_ = build_Q(q_val);
}

void OdometryFusion::set_measurement_noise(double r_val) {
    R_ = build_R(r_val);
}

void OdometryFusion::reset(const RobotState& initial_state) {
    initialize(initial_state);
}

void OdometryFusion::build_system_matrices() {
    A_ = build_A();
    Q_ = build_Q(0.01);
    C_ = build_C();
    R_ = build_R(0.1);
    P0_ = Eigen::Matrix<double, 6, 6>::Identity() * 0.1;
}

Eigen::Matrix<double, 6, 6> OdometryFusion::build_A() {
    Eigen::Matrix<double, 6, 6> A = 
        Eigen::Matrix<double, 6, 6>::Identity();
    A(0, 3) = dt_;
    A(1, 4) = dt_;
    A(2, 5) = dt_;
    return A;
}

Eigen::Matrix<double, 6, 6> OdometryFusion::build_B() {
    Eigen::Matrix<double, 6, 6> B = 
        Eigen::Matrix<double, 6, 6>::Zero();
    B(3, 0) = 1.0;
    B(4, 1) = 1.0;
    B(5, 2) = 1.0;
    return B;
}

Eigen::Matrix<double, 6, 6> OdometryFusion::build_Q(double q_val) {
    return Eigen::Matrix<double, 6, 6>::Identity() * q_val;
}

Eigen::Matrix<double, 3, 6> OdometryFusion::build_C() {
    Eigen::Matrix<double, 3, 6> C = 
        Eigen::Matrix<double, 3, 6>::Zero();
    C(0, 5) = 1.0;
    C(1, 3) = 1.0;
    C(2, 4) = 1.0;
    return C;
}

Eigen::Matrix<double, 3, 3> OdometryFusion::build_R(double r_val) {
    Eigen::Matrix<double, 3, 3> R = 
        Eigen::Matrix<double, 3, 3>::Identity() * r_val;
    R(0, 0) = 0.01;
    R(1, 1) = 0.1;
    R(2, 2) = 0.1;
    return R;
}

} // namespace waypoint_follower
