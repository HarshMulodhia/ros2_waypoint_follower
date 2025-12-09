#include "waypoint_follower/waypoint_follower_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "waypoints_default_path.hpp"

namespace waypoint_follower {

WaypointFollowerNode::WaypointFollowerNode()
    : Node("waypoint_follower_node"),
      state_(State::IDLE),
      current_waypoint_idx_(0) {

    RCLCPP_INFO(this->get_logger(), "Initializing Waypoint Follower Node");

    declare_parameter("controller_frequency", 50.0);
    declare_parameter("max_steering_angle", 0.5);
    declare_parameter("max_linear_velocity", 1.5);
    declare_parameter("waypoints_file", WAYPOINTS_DEFAULT_PATH);
    declare_parameter("goal_tolerance", 0.1);
    declare_parameter("steering_pid.kp", 1.0);
    declare_parameter("steering_pid.ki", 0.1);
    declare_parameter("steering_pid.kd", 0.5);
    declare_parameter("velocity_pid.kp", 0.5);
    declare_parameter("velocity_pid.ki", 0.05);
    declare_parameter("velocity_pid.kd", 0.1);

    double controller_freq = this->get_parameter("controller_frequency").as_double();
    double max_steering = this->get_parameter("max_steering_angle").as_double();
    double max_linear_vel = this->get_parameter("max_linear_velocity").as_double();
        std::string waypoints_file = this->get_parameter("waypoints_file").as_string();

        // If the configured path doesn't exist, try resolving relative to the
        // package's share directory so installed config files are found.
        namespace fs = std::filesystem;
        if (!fs::exists(waypoints_file)) {
            try {
                auto share = ament_index_cpp::get_package_share_directory("waypoint_follower");
                std::string candidate = share + "/" + waypoints_file;
                if (fs::exists(candidate)) {
                    waypoints_file = candidate;
                }
            } catch (const std::exception &e) {
                RCLCPP_DEBUG(this->get_logger(), "Could not resolve package share: %s", e.what());
            }
        }

    double dt = 1.0 / controller_freq;
    path_tracker_ = std::make_unique<PathTrackerController>(dt, max_steering, max_linear_vel);
    odometry_fusion_ = std::make_unique<OdometryFusion>(dt);
    trajectory_smoother_ = std::make_unique<TrajectorySmoother>();
    waypoint_loader_ = std::make_unique<WaypointLoader>();

    double steering_kp = this->get_parameter("steering_pid.kp").as_double();
    double steering_ki = this->get_parameter("steering_pid.ki").as_double();
    double steering_kd = this->get_parameter("steering_pid.kd").as_double();
    path_tracker_->set_steering_pid_gains(steering_kp, steering_ki, steering_kd);

    double velocity_kp = this->get_parameter("velocity_pid.kp").as_double();
    double velocity_ki = this->get_parameter("velocity_pid.ki").as_double();
    double velocity_kd = this->get_parameter("velocity_pid.kd").as_double();
    path_tracker_->set_velocity_pid_gains(velocity_kp, velocity_ki, velocity_kd);

    if (!waypoint_loader_->load_from_yaml(waypoints_file)) {
        RCLCPP_WARN(this->get_logger(), "Failed to load waypoints from %s", 
                   waypoints_file.c_str());
    }

    robot_state_.x = 0;
    robot_state_.y = 0;
    robot_state_.theta = 0;
    robot_state_.vx = 0;
    robot_state_.vy = 0;
    robot_state_.omega = 0;

    odometry_fusion_->initialize(robot_state_);

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 
                                                                      rclcpp::QoS(10));
    fused_odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry/fused", 
                                                                           rclcpp::QoS(10));
    path_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "path_visualization", rclcpp::QoS(10));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", rclcpp::QoS(10),
        std::bind(&WaypointFollowerNode::odom_callback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", rclcpp::QoS(10),
        std::bind(&WaypointFollowerNode::imu_callback, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<FollowWaypointsAction>(
        this, "follow_waypoints",
        std::bind(&WaypointFollowerNode::handle_goal, this, 
                 std::placeholders::_1, std::placeholders::_2),
        std::bind(&WaypointFollowerNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&WaypointFollowerNode::handle_accepted, this, std::placeholders::_1));

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / controller_freq)),
        std::bind(&WaypointFollowerNode::control_loop_callback, this));

    RCLCPP_INFO(this->get_logger(), "Waypoint Follower Node initialized successfully");
}

rclcpp_action::GoalResponse WaypointFollowerNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<const FollowWaypointsAction::Goal> goal) {

    RCLCPP_INFO(this->get_logger(), "Received goal with %zu waypoints", goal->poses.size());

    if (goal->poses.empty()) {
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WaypointFollowerNode::handle_cancel(
    const std::shared_ptr<ActionGoalHandle> goal_handle) {

    RCLCPP_INFO(this->get_logger(), "Goal canceled");
    state_ = State::IDLE;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void WaypointFollowerNode::handle_accepted(
    const std::shared_ptr<ActionGoalHandle> goal_handle) {

    current_goal_handle_ = goal_handle;
    state_ = State::PLANNING;
    current_waypoint_idx_ = 0;

    std::vector<Waypoint> waypoints;
    for (const auto& pose : goal_handle->get_goal()->poses) {
        Waypoint wp;
        wp.x = pose.pose.position.x;
        wp.y = pose.pose.position.y;
        wp.velocity = 0.5;
        waypoints.push_back(wp);
    }

    current_trajectory_ = trajectory_smoother_->smooth_waypoints(waypoints);
    trajectory_smoother_->generate_velocity_profile(current_trajectory_);
    publish_path_visualization(current_trajectory_);
    state_ = State::TRACKING;
}

void WaypointFollowerNode::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {

    robot_state_.x = msg->pose.pose.position.x;
    robot_state_.y = msg->pose.pose.position.y;
    robot_state_.vx = msg->twist.twist.linear.x;
    robot_state_.vy = msg->twist.twist.linear.y;
    robot_state_.omega = msg->twist.twist.angular.z;

    const auto& q = msg->pose.pose.orientation;
    robot_state_.theta = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    robot_state_.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
}

void WaypointFollowerNode::imu_callback(
    const sensor_msgs::msg::Imu::SharedPtr msg) {

    odometry_fusion_->update_from_imu(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->angular_velocity.z);
}

void WaypointFollowerNode::control_loop_callback() {
    if (state_ == State::IDLE || state_ == State::ERROR) {
        return;
    }

    if (current_trajectory_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No trajectory loaded");
        state_ = State::IDLE;
        return;
    }

    odometry_fusion_->predict_from_odometry(robot_state_.vx, robot_state_.omega);
    TrackingError error = compute_tracking_error();
    ControlCommand cmd = path_tracker_->update(robot_state_, error);

    publish_command(cmd);

    RobotState fused = odometry_fusion_->get_fused_state();
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = "odom";
    odom_msg.header.stamp = this->now();
    odom_msg.pose.pose.position.x = fused.x;
    odom_msg.pose.pose.position.y = fused.y;
    odom_msg.twist.twist.linear.x = fused.vx;
    odom_msg.twist.twist.angular.z = fused.omega;
    fused_odometry_pub_->publish(odom_msg);

    if (current_waypoint_idx_ >= current_trajectory_.points.size() - 1) {
        if (std::abs(error.cross_track_error) < 0.1) {
            state_ = State::GOAL_REACHED;
            if (current_goal_handle_) {
                auto result = std::make_shared<FollowWaypointsAction::Result>();
                current_goal_handle_->succeed(result);
                current_goal_handle_ = nullptr;
            }
            state_ = State::IDLE;
        }
    }
}

void WaypointFollowerNode::publish_command(const ControlCommand& cmd) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = cmd.linear_velocity;
    twist.linear.y = cmd.lateral_velocity;
    twist.angular.z = cmd.angular_velocity;
    cmd_vel_pub_->publish(twist);
}

void WaypointFollowerNode::publish_path_visualization(const Trajectory& trajectory) {
    visualization_msgs::msg::MarkerArray markers;

    for (size_t i = 0; i < trajectory.points.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = this->now();
        marker.ns = "trajectory";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = trajectory.points[i].x;
        marker.pose.position.y = trajectory.points[i].y;
        marker.pose.position.z = 0;

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.r = 0.0;
        marker.color.g = 0.5;
        marker.color.b = 1.0;
        marker.color.a = 0.8;

        markers.markers.push_back(marker);
    }

    path_viz_pub_->publish(markers);
}

TrackingError WaypointFollowerNode::compute_tracking_error() {
    TrackingError error;

    if (current_trajectory_.empty()) {
        return error;
    }

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

    current_waypoint_idx_ = nearest_idx;

    if (nearest_idx < current_trajectory_.points.size() - 1) {
        const auto& p1 = current_trajectory_.points[nearest_idx];
        const auto& p2 = current_trajectory_.points[nearest_idx + 1];

        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double path_length = std::hypot(dx, dy);

        if (path_length > 1e-6) {
            double rx = robot_state_.x - p1.x;
            double ry = robot_state_.y - p1.y;

            double cross = dx * ry - dy * rx;
            error.cross_track_error = cross / path_length;
        }
    }

    double path_heading = current_trajectory_.points[nearest_idx].theta;
    error.heading_error = math::angle_diff(robot_state_.theta, path_heading);

    double desired_velocity = current_trajectory_.points[nearest_idx].velocity;
    error.velocity_error = desired_velocity - std::hypot(robot_state_.vx, robot_state_.vy);

    error.progress = static_cast<double>(nearest_idx) /
                     current_trajectory_.points.size();

    return error;
}

}  // namespace waypoint_follower
