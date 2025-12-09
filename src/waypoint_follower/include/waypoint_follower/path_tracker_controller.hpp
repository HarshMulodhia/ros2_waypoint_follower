#pragma once

#include "waypoint_types.hpp"
#include <memory>
#include <control/pid_controller.hpp>
#include <control/lqr_controller.hpp>

namespace waypoint_follower {

/**
 * @class PathTrackerController
 * @brief Path tracking control using PID and optional LQR
 * @param dt Time step (seconds)
 * @param max_steering Maximum steering angle (radians)
 * @param max_linear_vel Maximum linear velocity (m/s)
 */
class PathTrackerController {
public:
    /// Constructor
    PathTrackerController(double dt, double max_steering, double max_linear_vel);
    ~PathTrackerController() = default;

    /// Update control command
    ControlCommand update(const RobotState& state, const TrackingError& error);

    /// Set steering PID gains
    void set_steering_pid_gains(double kp, double ki, double kd);

    /// Set velocity PID gains
    void set_velocity_pid_gains(double kp, double ki, double kd);

    /// Enable/disable LQR
    void enable_lqr(bool enable) { use_lqr_ = enable; }

    /// Check if LQR enabled
    bool is_lqr_enabled() const { return use_lqr_; }

    /// Reset controller state
    void reset();

private:
    double dt_;
    double max_steering_angle_;
    double max_linear_velocity_;
    bool use_lqr_;

    std::unique_ptr<robotics_control::PIDController> steering_pid_;
    std::unique_ptr<robotics_control::PIDController> velocity_pid_;
    std::unique_ptr<robotics_control::LQRController> lqr_controller_;

    double compute_lookahead_distance(double velocity);
    Eigen::Vector2d compute_lookahead_point(const RobotState& state, const Trajectory& trajectory);
};

}
