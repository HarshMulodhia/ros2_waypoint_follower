/**
 * @file path_tracker_controller.hpp
 * @brief Path tracking controller header
 * @author Autonomous Vehicle Team
 * @date December 2025
 * @version 2.0.1
 */

#pragma once

#include "waypoint_types.hpp"
#include <memory>
#include <Eigen/Dense>

namespace waypoint_follower {

class PathTrackerController {
public:
    /**
     * @brief Constructor
     * @param dt Time step in seconds
     * @param max_steering Maximum steering angle
     * @param max_linear_vel Maximum linear velocity
     */
    PathTrackerController(double dt, double max_steering, double max_linear_vel);

    /**
     * @brief Update control command based on tracking error
     * @param state Current robot state
     * @param error Current tracking error
     * @return Control command (velocity and steering)
     */
    ControlCommand update(const RobotState& state, const TrackingError& error);

    /**
     * @brief Set steering PID gains
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void set_steering_pid_gains(double kp, double ki, double kd);

    /**
     * @brief Set velocity PID gains
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void set_velocity_pid_gains(double kp, double ki, double kd);

    /**
     * @brief Reset controller state
     */
    void reset();

    /**
     * @brief Compute lookahead distance based on velocity
     * @param velocity Current velocity
     * @return Lookahead distance
     */
    double compute_lookahead_distance(double velocity);

    /**
     * @brief Compute lookahead point on trajectory
     * @param state Current robot state
     * @param trajectory Reference trajectory
     * @return Lookahead point
     */
    Eigen::Vector2d compute_lookahead_point(
        const RobotState& state, const Trajectory& trajectory);

private:
    double dt_;
    double max_steering_angle_;
    double max_linear_velocity_;
    bool use_lqr_;

    // PID controller state variables
    double steering_integral_;
    double steering_prev_error_;
    double velocity_integral_;
    double velocity_prev_error_;

    // PID gains
    double steering_kp_;
    double steering_ki_;
    double steering_kd_;
    double velocity_kp_;
    double velocity_ki_;
    double velocity_kd_;
};

} // namespace waypoint_follower
