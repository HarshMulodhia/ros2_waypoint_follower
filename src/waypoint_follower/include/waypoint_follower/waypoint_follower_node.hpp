#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "odometry_fusion.hpp"
#include "path_tracker_controller.hpp"
#include "trajectory_smoother.hpp"
#include "waypoint_loader.hpp"
#include "waypoint_types.hpp"

namespace waypoint_follower {

enum class State {
  IDLE = 0,
  PLANNING = 1,
  TRACKING = 2,
  GOAL_REACHED = 3,
  ERROR = 4
};

class WaypointFollowerNode : public rclcpp::Node {
public:
  WaypointFollowerNode();

private:
  using FollowWaypointsAction = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints =
      rclcpp_action::ServerGoalHandle<FollowWaypointsAction>;

  double goal_tolerance_distance_;

  // Action handlers
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const FollowWaypointsAction::Goal> goal);

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleFollowWaypoints> goal_handle);

  void
  handle_accepted(const std::shared_ptr<GoalHandleFollowWaypoints> goal_handle);

  // Subscription callbacks
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // Control loop
  void control_loop_callback();

  // Helper functions
  TrackingError compute_tracking_error();
  void publish_command(const ControlCommand &cmd);
  void publish_path_visualization(const Trajectory &trajectory);

  // Components
  std::unique_ptr<WaypointLoader> waypoint_loader_;
  std::unique_ptr<PathTrackerController> path_tracker_;
  std::unique_ptr<OdometryFusion> odometry_fusion_;
  std::unique_ptr<TrajectorySmoother> trajectory_smoother_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odometry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_viz_pub_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Action server
  rclcpp_action::Server<FollowWaypointsAction>::SharedPtr action_server_;

  // Timer for control loop
  rclcpp::TimerBase::SharedPtr control_timer_;

  // State
  State state_;
  RobotState robot_state_;
  Trajectory current_trajectory_;
  size_t current_waypoint_idx_;
  std::shared_ptr<GoalHandleFollowWaypoints> current_goal_handle_;
};

} // namespace waypoint_follower
