/**
 * @file waypoint_follower_node.cpp
 * @brief Main ROS2 node for waypoint following
 * @author Autonomous Vehicle Team
 * @date December 2025
 * @version 2.0.1 - FIXED
 */

#include "waypoint_follower/waypoint_follower_node.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace waypoint_follower {

WaypointFollowerNode::WaypointFollowerNode()
    : Node("waypoint_follower_node"), 
      state_(State::IDLE),
      current_waypoint_idx_(0) {
    
    RCLCPP_INFO(get_logger(), "Initializing WaypointFollowerNode...");
    
    // Initialize components
    waypoint_loader_ = std::make_unique<WaypointLoader>();
    path_tracker_ = std::make_unique<PathTrackerController>(0.02, 0.5, 1.5);
    odometry_fusion_ = std::make_unique<OdometryFusion>(0.02);
    trajectory_smoother_ = std::make_unique<TrajectorySmoother>(0.5);
    
    // Initialize robot state
    robot_state_.x = 0.0;
    robot_state_.y = 0.0;
    robot_state_.theta = 0.0;
    robot_state_.vx = 0.0;
    robot_state_.vy = 0.0;
    robot_state_.omega = 0.0;
    robot_state_.timestamp = 0.0;
    
    // Create publishers
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::QoS(10));
    
    fused_odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        "odometry/fused", rclcpp::QoS(10));
    
    path_viz_pub_ = create_publisher<nav_msgs::msg::Path>(
        "path_visualization", rclcpp::QoS(10));
    
    // Create subscribers
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::QoS(10),
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
            this->odom_callback(msg);
        });
    
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", rclcpp::QoS(10),
        [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            this->imu_callback(msg);
        });
    
    // Create action server
    action_server_ = rclcpp_action::create_server<FollowWaypointsAction>(
        this,
        "follow_waypoints",
        std::bind(&WaypointFollowerNode::handle_goal, this, 
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&WaypointFollowerNode::handle_cancel, this, 
                  std::placeholders::_1),
        std::bind(&WaypointFollowerNode::handle_accepted, this, 
                  std::placeholders::_1));
    
    // Create control timer (50 Hz)
    control_timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        [this]() { this->control_loop_callback(); });
    
    RCLCPP_INFO(get_logger(), "WaypointFollowerNode initialized successfully");
}

rclcpp_action::GoalResponse WaypointFollowerNode::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FollowWaypointsAction::Goal> goal) {
    
    RCLCPP_INFO(get_logger(), "Received goal with %zu waypoints",
               goal->poses.size());
    
    if (goal->poses.empty()) {
        RCLCPP_WARN(get_logger(), "Empty waypoint list");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    state_ = State::PLANNING;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WaypointFollowerNode::handle_cancel(
    const std::shared_ptr<GoalHandleFollowWaypoints>) {
    
    RCLCPP_INFO(get_logger(), "Received cancel request");
    state_ = State::IDLE;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void WaypointFollowerNode::handle_accepted(
    const std::shared_ptr<GoalHandleFollowWaypoints> goal_handle) {
    
    current_goal_handle_ = goal_handle;
    
    // Load waypoints from goal
    waypoint_loader_->clear();
    const auto& poses = goal_handle->get_goal()->poses;
    
    for (const auto& pose : poses) {
        Waypoint wp;
        wp.x = pose.pose.position.x;
        wp.y = pose.pose.position.y;
        wp.velocity = 0.5;
        waypoint_loader_->add_waypoint(wp);
    }
    
    if (!waypoint_loader_->validate()) {
        goal_handle->abort(std::make_shared<FollowWaypointsAction::Result>());
        state_ = State::IDLE;
    }
}

void WaypointFollowerNode::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    
    robot_state_.x = msg->pose.pose.position.x;
    robot_state_.y = msg->pose.pose.position.y;
    robot_state_.vx = msg->twist.twist.linear.x;
    robot_state_.vy = msg->twist.twist.linear.y;
    robot_state_.omega = msg->twist.twist.angular.z;
    robot_state_.timestamp = msg->header.stamp.sec + 
                            msg->header.stamp.nanosec * 1e-9;
    
    // Extract theta from quaternion
    double w = msg->pose.pose.orientation.w;
    double z = msg->pose.pose.orientation.z;
    robot_state_.theta = 2.0 * std::atan2(z, w);
}

void WaypointFollowerNode::imu_callback(
    const sensor_msgs::msg::Imu::SharedPtr msg) {
    
    odometry_fusion_->update_from_imu(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->angular_velocity.z);
}

void WaypointFollowerNode::control_loop_callback() {
    switch (state_) {
        case State::IDLE:
            break;
            
        case State::PLANNING: {
            // Generate smooth trajectory
            current_trajectory_ = trajectory_smoother_->smooth_waypoints(
                waypoint_loader_->get_waypoints(), 0.1);
            
            if (!current_trajectory_.empty()) {
                publish_path_visualization(current_trajectory_);
                state_ = State::TRACKING;
                current_waypoint_idx_ = 0;
                RCLCPP_INFO(get_logger(), "Starting trajectory tracking");
            } else {
                state_ = State::ERROR;
                RCLCPP_ERROR(get_logger(), "Failed to generate trajectory");
            }
            break;
        }
            
        case State::TRACKING: {
            // Compute tracking error
            TrackingError error = compute_tracking_error();
            
            // Update control
            ControlCommand cmd = path_tracker_->update(robot_state_, error);
            
            // Publish command
            publish_command(cmd);
            
            // Check goal reached
            double dist_to_goal = std::hypot(
                waypoint_loader_->get_waypoints().back().x - robot_state_.x,
                waypoint_loader_->get_waypoints().back().y - robot_state_.y);
            
            if (dist_to_goal < 0.1) {
                state_ = State::GOAL_REACHED;
                RCLCPP_INFO(get_logger(), "Goal reached!");
            }
            break;
        }
            
        case State::GOAL_REACHED: {
            // Stop robot
            ControlCommand stop_cmd;
            publish_command(stop_cmd);
            
            // Signal action completion
            if (current_goal_handle_) {
                auto result = std::make_shared<FollowWaypointsAction::Result>();
                current_goal_handle_->succeed(result);
            }
            state_ = State::IDLE;
            break;
        }
            
        case State::ERROR: {
            // Stop robot
            ControlCommand stop_cmd;
            publish_command(stop_cmd);
            
            if (current_goal_handle_) {
                current_goal_handle_->abort(std::make_shared<FollowWaypointsAction::Result>());
            }
            state_ = State::IDLE;
            break;
        }
    }
}

TrackingError WaypointFollowerNode::compute_tracking_error() {
    TrackingError error;
    
    if (current_trajectory_.empty()) {
        return error;
    }
    
    // Find nearest point on trajectory
    size_t nearest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < current_trajectory_.points.size(); ++i) {
        double dist = std::hypot(
            current_trajectory_.points[i].x - robot_state_.x,
            current_trajectory_.points[i].y - robot_state_.y);
        
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
    }
    
    // Cross-track error
    const auto& nearest = current_trajectory_.points[nearest_idx];
    error.cross_track_error = min_dist;
    
    // Heading error
    error.heading_error = nearest.theta - robot_state_.theta;
    while (error.heading_error > M_PI) error.heading_error -= 2 * M_PI;
    while (error.heading_error < -M_PI) error.heading_error += 2 * M_PI;
    
    // Velocity error
    error.velocity_error = nearest.velocity - robot_state_.vx;
    
    // Progress
    error.progress = nearest.s / current_trajectory_.total_length;
    
    return error;
}

void WaypointFollowerNode::publish_command(const ControlCommand& cmd) {
    auto msg = std::make_unique<geometry_msgs::msg::Twist>();
    msg->linear.x = cmd.linear_velocity;
    msg->linear.y = cmd.lateral_velocity;
    msg->angular.z = cmd.angular_velocity;
    
    cmd_vel_pub_->publish(std::move(msg));
}

void WaypointFollowerNode::publish_path_visualization(
    const Trajectory& trajectory) {
    
    auto msg = std::make_unique<nav_msgs::msg::Path>();
    msg->header.frame_id = "map";
    msg->header.stamp = get_clock()->now();
    
    for (const auto& point : trajectory.points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.position.z = 0.0;
        
        // Convert theta to quaternion
        double half_theta = point.theta / 2.0;
        pose.pose.orientation.w = std::cos(half_theta);
        pose.pose.orientation.z = std::sin(half_theta);
        
        msg->poses.push_back(pose);
    }
    
    path_viz_pub_->publish(std::move(msg));
}

} // namespace waypoint_follower
