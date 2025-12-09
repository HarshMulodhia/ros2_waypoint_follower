#pragma once

#include "waypoint_types.hpp"
#include "waypoint_loader.hpp"
#include "path_tracker_controller.hpp"
#include "odometry_fusion.hpp"
#include "trajectory_smoother.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace waypoint_follower {

/// @class WaypointFollowerNode
/// @brief Main ROS2 node for waypoint following
class WaypointFollowerNode : public rclcpp::Node {
public:
    WaypointFollowerNode();
    ~WaypointFollowerNode() = default;

private:
    enum class State {
        IDLE,
        PLANNING,
        TRACKING,
        GOAL_REACHED,
        ERROR
    };

    std::unique_ptr<WaypointLoader> waypoint_loader_;
    std::unique_ptr<PathTrackerController> path_tracker_;
    std::unique_ptr<OdometryFusion> odometry_fusion_;
    std::unique_ptr<TrajectorySmoother> trajectory_smoother_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odometry_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_viz_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    using FollowWaypointsAction = nav2_msgs::action::FollowWaypoints;
    using ActionGoalHandle = rclcpp_action::ServerGoalHandle<FollowWaypointsAction>;
    rclcpp_action::Server<FollowWaypointsAction>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        const std::shared_ptr<const FollowWaypointsAction::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<ActionGoalHandle> goal_handle);

    void handle_accepted(const std::shared_ptr<ActionGoalHandle> goal_handle);

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void control_loop_callback();

    void publish_command(const ControlCommand& cmd);
    void publish_path_visualization(const Trajectory& trajectory);
    TrackingError compute_tracking_error();

    State state_;
    RobotState robot_state_;
    Trajectory current_trajectory_;
    size_t current_waypoint_idx_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    std::shared_ptr<ActionGoalHandle> current_goal_handle_;
};

}
