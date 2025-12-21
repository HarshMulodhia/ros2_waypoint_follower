/**
 * @file waypoint_follower_node.hpp
 * @brief Main ROS 2 node for waypoint following system
 * @author Harsh Mulodhia
 * @version 1.0.0
 *
 * @class WaypointFollowerNode
 * @brief Orchestrates complete autonomous navigation system
 *
 * Responsibilities:
 * - ROS 2 action server for goal reception (/follow_waypoints)
 * - Component initialization and lifecycle management
 * - Finite state machine implementation
 * - 50 Hz control loop execution
 * - Multi-sensor subscription handling
 * - Command publishing and feedback
 *
 * **Subscribed Topics:**
 * - /odom: Current robot odometry
 * - /imu/data: IMU sensor data
 *
 * **Published Topics:**
 * - /cmd_vel: Velocity commands to robot
 * - /odometry/fused: Fused state estimate
 * - /path_visualization: Trajectory for RViz
 *
 * **Action Server:**
 * - /follow_waypoints: nav2_msgs/FollowWaypoints interface
 *
 * **State Machine:**
 * @code
 * IDLE → PLANNING → TRACKING → GOAL_REACHED → IDLE
 *  ↑                     ↓
 *  └─────── ERROR ───────┘
 * @endcode
 *
 * @see control_loop_callback() for main control logic
 * @see handle_goal(), handle_cancel(), handle_accepted() for action handling
 */

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

  /**
   * Initialize waypoint follower node
   * Starts at IDLE state
   * Creates:
   *   - Publishers: /cmd_vel, /odometry/fused, /path_visualization
   *   - Subscribers: /odom, /imu/data
   *   - Action server: /follow_waypoints
   *   - Control timer: 50 Hz
   */
  WaypointFollowerNode();

private:

  /// ROS 2 Action: nav2_msgs/FollowWaypoints
  /// Server name: /follow_waypoints
  /// Goal: std::vector<geometry_msgs::msg::PoseStamped> poses
  /// Result: std::vector<size_t> missed_waypoints (empty if successful)
  /// Feedback: (none currently implemented)

  using FollowWaypointsAction = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints =
      rclcpp_action::ServerGoalHandle<FollowWaypointsAction>;

  double goal_tolerance_distance_;

  // Action handlers

  /**
   * Validate goal and decide whether to accept
   * @param uuid Unique ID for this goal
   * @param goal Goal request with pose list
   * @return GoalResponse::ACCEPT_AND_EXECUTE if poses non-empty
   *         GoalResponse::REJECT if poses empty
   */
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const FollowWaypointsAction::Goal> goal);

  /**
   * Handle goal cancellation
   * @param goal_handle Handle to running goal
   * @return CancelResponse::ACCEPT (always cancel)
   * @effects Publishes v=0, transitions to IDLE
   */
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFollowWaypoints> goal_handle);

  /**
   * Execute accepted goal
   * @param goal_handle Handle to running goal
   * @effects Transitions to PLANNING state, loads waypoints
   */
  void handle_accepted(const std::shared_ptr<GoalHandleFollowWaypoints> goal_handle);

  // Subscription callbacks

  /**
   * Odometry callback: receive /odom messages
   * @param msg nav_msgs::Odometry message
   * @effects Updates robot_state_ with pose and velocity
   * @note Extracts yaw from quaternion: θ = 2*atan2(qz, qw)
   */
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * IMU callback: receive /imu/data messages
   * @param msg sensor_msgs::Imu message
   * @effects Passes accelerations and gyro to odometry_fusion_
   */
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // Control loop

  /**
   * Main control loop callback (50 Hz timer)
   * Executes state machine and generates commands
   * 
   * IDLE: publish v=0
   * 
   * PLANNING:
   *   - Call trajectory_smoother_->smooth_waypoints()
   *   - Publish trajectory via path_viz_pub_
   *   - Transition to TRACKING if successful
   * 
   * TRACKING:
   *   - Compute tracking error
   *   - Call path_tracker_->update()
   *   - Publish command via cmd_vel_pub_
   *   - Check goal condition:
   *     * progress > 99% AND distance_to_goal < 0.2m
   *   - Transition to GOAL_REACHED
   * 
   * GOAL_REACHED:
   *   - Publish v=0
   *   - Call goal_handle->succeed()
   *   - Transition to IDLE
   * 
   * ERROR:
   *   - Publish v=0
   *   - Call goal_handle->abort()
   *   - Transition to IDLE
   */
  void control_loop_callback();

  // Helper functions

  /**
   * Compute tracking error metrics
   * @return TrackingError struct with CTE, heading error, velocity error, progress
   * 
   * Algorithm:
   *   1. Find nearest trajectory point to robot
   *   2. Use next point for path heading
   *   3. Cross-track error: perpendicular distance to path
   *   4. Heading error: difference between path heading and robot heading
   *   5. Velocity error: target velocity - current velocity
   *   6. Progress: (closest_idx / total_points) ∈ [0, 1]
   */
  TrackingError compute_tracking_error();

  /**
   * Publish velocity command to /cmd_vel
   * @param cmd ControlCommand with linear and angular velocities
   */
  void publish_command(const ControlCommand &cmd);

  /**
   * Publish trajectory for visualization in RViz
   * @param trajectory Smoothed trajectory
   * Publishes: nav_msgs::Path with all trajectory points
   */
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

  // State management
  State state_;                                 ///< Current FSM state
  RobotState robot_state_;                      ///< Latest estimated state
  Trajectory current_trajectory_;               ///< Active trajectory
  size_t current_waypoint_idx_;                 ///< Progress through waypoints
  std::shared_ptr<GoalHandleFollowWaypoints> current_goal_handle_;
};

} // namespace waypoint_follower
