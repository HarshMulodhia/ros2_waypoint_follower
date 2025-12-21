#include "waypoint_follower/waypoint_follower_node.hpp"

#include "waypoint_follower/odometry_fusion.hpp"
#include "waypoint_follower/path_tracker_controller.hpp"
#include "waypoint_follower/trajectory_smoother.hpp"
#include "waypoint_follower/waypoint_loader.hpp"

#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace waypoint_follower {

WaypointFollowerNode::WaypointFollowerNode()
    : Node("waypoint_follower_node"), state_(State::IDLE),
      current_waypoint_idx_(0) {
  RCLCPP_INFO(get_logger(), "Initializing WaypointFollowerNode...");

  // Components
  waypoint_loader_ = std::make_unique<WaypointLoader>();
  path_tracker_ = std::make_unique<PathTrackerController>(0.02, 0.5, 1.5);
  odometry_fusion_ = std::make_unique<OdometryFusion>(0.02);
  trajectory_smoother_ = std::make_unique<TrajectorySmoother>(0.5);

  // Parameters (optional YAML fallback)
  const std::string waypoints_file =
      this->declare_parameter("waypoints_file", "waypoints_course.yaml");
  if (!waypoint_loader_->load_from_yaml(waypoints_file)) {
    RCLCPP_WARN(get_logger(),
                "Failed to load waypoints from '%s' (continuing).",
                waypoints_file.c_str());
  }

  // Publishers
  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  fused_odometry_pub_ =
      this->create_publisher<nav_msgs::msg::Odometry>("odometry/fused", 10);
  path_viz_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("path_visualization", 10);

  // Subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&WaypointFollowerNode::odom_callback, this,
                std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", 10,
      std::bind(&WaypointFollowerNode::imu_callback, this,
                std::placeholders::_1));

  // Action server
  RCLCPP_INFO(get_logger(), "Creating action server: /follow_waypoints");
  action_server_ = rclcpp_action::create_server<FollowWaypointsAction>(
      this, "follow_waypoints",
      std::bind(&WaypointFollowerNode::handle_goal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&WaypointFollowerNode::handle_cancel, this,
                std::placeholders::_1),
      std::bind(&WaypointFollowerNode::handle_accepted, this,
                std::placeholders::_1));

  // Control loop timer (50 Hz)
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&WaypointFollowerNode::control_loop_callback, this));

  RCLCPP_INFO(get_logger(), "WaypointFollowerNode initialized successfully");
}

// =======================
// Action callbacks
// =======================

rclcpp_action::GoalResponse WaypointFollowerNode::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FollowWaypointsAction::Goal> goal) {
  RCLCPP_INFO(get_logger(), "handle_goal(): poses=%zu", goal->poses.size());
  if (goal->poses.empty()) {
    RCLCPP_ERROR(get_logger(), "handle_goal(): REJECT (empty poses)");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(get_logger(), "handle_goal(): ACCEPT_AND_EXECUTE");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WaypointFollowerNode::handle_cancel(
    const std::shared_ptr<GoalHandleFollowWaypoints> goal_handle) {
  (void)goal_handle;
  RCLCPP_INFO(get_logger(), "handle_cancel(): ACCEPT");
  publish_command(ControlCommand(0.0, 0.0, 0.0));
  state_ = State::IDLE;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void WaypointFollowerNode::handle_accepted(
    const std::shared_ptr<GoalHandleFollowWaypoints> goal_handle) {
  RCLCPP_INFO(get_logger(), "handle_accepted(): ENTER");
  current_goal_handle_ = goal_handle;

  // Convert goal poses into internal waypoint list
  waypoint_loader_->load_from_poses(goal_handle->get_goal()->poses);

  // Validate that waypoints were loaded
  if (waypoint_loader_->get_waypoints().empty()) {
    RCLCPP_ERROR(get_logger(), "handle_accepted(): No waypoints in goal");
    auto result = std::make_shared<FollowWaypointsAction::Result>();
    current_goal_handle_->abort(result);
    state_ = State::ERROR;
    return;
  }

  state_ = State::PLANNING;
  current_waypoint_idx_ = 0;
  RCLCPP_INFO(get_logger(),
              "handle_accepted(): state_=PLANNING with %zu waypoints",
              waypoint_loader_->get_waypoints().size());
}

// =======================
// Sub callbacks
// =======================

void WaypointFollowerNode::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_state_.x = msg->pose.pose.position.x;
  robot_state_.y = msg->pose.pose.position.y;
  robot_state_.vx = msg->twist.twist.linear.x;
  robot_state_.vy = msg->twist.twist.linear.y;
  robot_state_.omega = msg->twist.twist.angular.z;

  // planar yaw extraction (w,z)
  const double w = msg->pose.pose.orientation.w;
  const double z = msg->pose.pose.orientation.z;
  robot_state_.theta = 2.0 * std::atan2(z, w);
}

void WaypointFollowerNode::imu_callback(
    const sensor_msgs::msg::Imu::SharedPtr msg) {
  odometry_fusion_->update_from_imu(msg->linear_acceleration.x,
                                    msg->linear_acceleration.y,
                                    msg->angular_velocity.z);
}

// =======================
// Control loop / FSM
// =======================

void WaypointFollowerNode::control_loop_callback() {
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                       "control_loop: state=%d", static_cast<int>(state_));

  switch (state_) {
  case State::IDLE: {
    publish_command(ControlCommand(0.0, 0.0, 0.0));
    break;
  }

  case State::PLANNING: {
    current_trajectory_ = trajectory_smoother_->smooth_waypoints(
        waypoint_loader_->get_waypoints(), 0.1);

    if (!current_trajectory_.empty()) {
      publish_path_visualization(current_trajectory_);
      current_waypoint_idx_ = 0;
      state_ = State::TRACKING;
      RCLCPP_INFO(get_logger(), "PLANNING -> TRACKING (%zu trajectory points)",
                  current_trajectory_.points.size());
    } else {
      state_ = State::ERROR;
      RCLCPP_ERROR(get_logger(), "PLANNING failed: empty trajectory");
    }
    break;
  }

  case State::TRACKING: {
    TrackingError error = compute_tracking_error();
    ControlCommand cmd = path_tracker_->update(robot_state_, error);
    publish_command(cmd);

    const auto &wpts = waypoint_loader_->get_waypoints();
    if (wpts.empty()) {
      state_ = State::ERROR;
      RCLCPP_ERROR(get_logger(), "TRACKING error: no waypoints");
      break;
    }

    const auto &goal_wp = wpts.back();
    const double dist_to_goal =
        std::hypot(goal_wp.x - robot_state_.x, goal_wp.y - robot_state_.y);

    // Require both: close to final goal AND near the end of the trajectory.
    const bool near_end_of_path = (error.progress > 0.99);

    if (near_end_of_path && dist_to_goal < 0.2) {
      state_ = State::GOAL_REACHED;
      RCLCPP_INFO(get_logger(), "Goal reached! progress=%.3f dist=%.3f",
                  error.progress, dist_to_goal);
    }

    break;
  }

  case State::GOAL_REACHED: {
    publish_command(ControlCommand(0.0, 0.0, 0.0));

    if (current_goal_handle_) {
      auto result = std::make_shared<FollowWaypointsAction::Result>();
      result->missed_waypoints.clear();
      current_goal_handle_->succeed(result);
      RCLCPP_INFO(get_logger(), "Action SUCCEEDED");
    }

    state_ = State::IDLE;
    break;
  }

  case State::ERROR: {
    publish_command(ControlCommand(0.0, 0.0, 0.0));

    if (current_goal_handle_) {
      auto result = std::make_shared<FollowWaypointsAction::Result>();
      current_goal_handle_->abort(result);
      RCLCPP_ERROR(get_logger(), "Action ABORTED due to error");
    }

    state_ = State::IDLE;
    break;
  }

  default: {
    RCLCPP_WARN(get_logger(), "Unknown state %d -> IDLE",
                static_cast<int>(state_));
    state_ = State::IDLE;
    break;
  }
  }
}

// =======================
// Helpers
// =======================

void WaypointFollowerNode::publish_command(const ControlCommand &cmd) {
  geometry_msgs::msg::Twist msg;
  msg.linear.x = cmd.linear_velocity;
  msg.angular.z = cmd.angular_velocity;
  cmd_vel_pub_->publish(msg);
}

void WaypointFollowerNode::publish_path_visualization(
    const Trajectory &trajectory) {
  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "odom";
  path.poses.reserve(trajectory.points.size());

  for (const auto &p : trajectory.points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = p.x;
    pose.pose.position.y = p.y;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  path_viz_pub_->publish(path);
}

TrackingError WaypointFollowerNode::compute_tracking_error() {
  TrackingError error{};
  if (current_trajectory_.points.empty()) {
    return error;
  }

  // Closest point on trajectory
  size_t closest_idx = 0;
  double min_dist_sq = std::numeric_limits<double>::max();
  for (size_t i = 0; i < current_trajectory_.points.size(); ++i) {
    const double dx = current_trajectory_.points[i].x - robot_state_.x;
    const double dy = current_trajectory_.points[i].y - robot_state_.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < min_dist_sq) {
      min_dist_sq = d2;
      closest_idx = i;
    }
  }

  const size_t lookahead_idx =
      std::min(closest_idx + 1, current_trajectory_.points.size() - 1);
  const auto &p0 = current_trajectory_.points[closest_idx];
  const auto &p1 = current_trajectory_.points[lookahead_idx];
  const double path_heading = std::atan2(p1.y - p0.y, p1.x - p0.x);

  // Cross-track error (perpendicular distance from path)
  const double dx = robot_state_.x - p0.x;
  const double dy = robot_state_.y - p0.y;
  error.cross_track_error =
      -dx * std::sin(path_heading) + dy * std::cos(path_heading);

  // Heading error
  error.heading_error = path_heading - robot_state_.theta;
  while (error.heading_error > M_PI)
    error.heading_error -= 2 * M_PI;
  while (error.heading_error < -M_PI)
    error.heading_error += 2 * M_PI;

  // Velocity error
  error.velocity_error = p0.velocity - robot_state_.vx;

  // Progress along trajectory
  error.progress = static_cast<double>(closest_idx) /
                   static_cast<double>(current_trajectory_.points.size());

  return error;
}

} // namespace waypoint_follower
