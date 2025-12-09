#pragma once

#include "waypoint_types.hpp"
#include <memory>
#include <Eigen/Dense>
#include <estimation/kalman_filter.hpp>

namespace waypoint_follower {

/**
 * @class OdometryFusion
 * @brief Multi-sensor odometry fusion using Kalman Filter
 */
class OdometryFusion {
public:
    /// Constructor
    OdometryFusion(double dt);
    ~OdometryFusion() = default;

    /// Initialize filter
    void initialize(const RobotState& initial_state);

    /// Predict from odometry
    void predict_from_odometry(double linear_vel, double angular_vel);

    /// Update from IMU
    void update_from_imu(double accel_x, double accel_y, double gyro_z);

    /// Update from encoders
    void update_from_encoder(double left_ticks, double right_ticks,
                            double wheel_base, double wheel_radius);

    /// Get fused state
    RobotState get_fused_state() const { return fused_state_; }

    /// Set process noise
    void set_process_noise(double q_val);

    /// Set measurement noise
    void set_measurement_noise(double r_val);

    /// Reset filter
    void reset(const RobotState& initial_state);

private:
    double dt_;
    RobotState fused_state_;
    std::unique_ptr<robotics_control::KalmanFilter> filter_;

    Eigen::Matrix<double, 6, 6> A_, Q_, P0_, B_;
    Eigen::Matrix<double, 3, 6> C_;
    Eigen::Matrix<double, 3, 3> R_;

    void build_system_matrices();
    Eigen::Matrix<double, 6, 6> build_A();
    Eigen::Matrix<double, 6, 6> build_B();
    Eigen::Matrix<double, 6, 6> build_Q(double q_val);
    Eigen::Matrix<double, 3, 6> build_C();
    Eigen::Matrix<double, 3, 3> build_R(double r_val);
};

}
