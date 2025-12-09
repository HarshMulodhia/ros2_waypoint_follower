#pragma once

#include <vector>
#include <cmath>
#include <Eigen/Dense>

namespace waypoint_follower {

/**
 * @struct Waypoint
 * @brief Single waypoint with position, orientation, and velocity
 * @param x X position (meters)
 * @param y Y position (meters)
 * @param theta Desired heading (radians)
 * @param velocity Desired velocity (m/s)
 */
struct Waypoint {
    double x, y, theta, velocity;

    Waypoint() : x(0), y(0), theta(0), velocity(0.5) {}
    Waypoint(double x_, double y_, double v_ = 0.5, double t_ = 0)
        : x(x_), y(y_), theta(t_), velocity(v_) {}
};

/**
 * @struct RobotState
 * @brief Complete robot state estimate [x, y, theta, vx, vy, omega]
 * @param x X position (meters)
 * @param y Y position (meters)
 * @param theta Heading angle (radians)
 * @param vx X velocity (m/s)
 * @param vy Y velocity (m/s)
 * @param omega Angular velocity (rad/s)
 * @param timestamp Timestamp (seconds)
 */
struct RobotState {
    double x, y, theta, vx, vy, omega, timestamp;

    RobotState() : x(0), y(0), theta(0), vx(0), vy(0), omega(0), timestamp(0) {}
    
    Eigen::Vector3d position() const {return Eigen::Vector3d(x, y, theta);}
    Eigen::Vector3d velocity() const {return Eigen::Vector3d(vx, vy, omega);}
};

/**
 * @struct ControlCommand
 * @brief Control output [linear_velocity, lateral_velocity, angular_velocity]
 * @param linear_velocity Forward velocity (m/s)
 * @param lateral_velocity Lateral velocity (m/s)
 * @param angular_velocity Angular velocity (rad/s)
 */
struct ControlCommand {
    double linear_velocity, lateral_velocity, angular_velocity;

    ControlCommand() : linear_velocity(0), lateral_velocity(0), angular_velocity(0) {}
    ControlCommand(double lin, double lat, double ang)
        : linear_velocity(lin), lateral_velocity(lat), angular_velocity(ang) {}
};

/// @struct TrackingError
/// @brief Error metrics for path tracking
/**
 * @struct TrackingError
 * @brief Error metrics for path tracking
 * @param cross_track_error Lateral error (meters)
 * @param heading_error Heading error (radians)
 * @param velocity_error Velocity error (m/s)
 * @param progress Progress [0, 1]
 */
struct TrackingError {
    double cross_track_error, heading_error, velocity_error, progress;
    TrackingError() : cross_track_error(0), heading_error(0), velocity_error(0), progress(0) {}
};

/**
 * @struct TrajectoryPoint
 * @brief Single point on trajectory
 * @param x X position (meters)
 * @param y Y position (meters)
 * @param theta Heading (radians)
 * @param curvature Path curvature (1/meters)
 * @param velocity Desired velocity (m/s)
 * @param s Arc length (meters)
 */
struct TrajectoryPoint {
    double x, y, theta, curvature, velocity, s;
    TrajectoryPoint() : x(0), y(0), theta(0), curvature(0), velocity(0.5), s(0) {}
};

/**
 * @struct Trajectory
 * @brief Complete smoothed trajectory
 * @param points Trajectory points
 * @param total_length Total path length (meters)
 */
struct Trajectory {
    std::vector<TrajectoryPoint> points;
    double total_length;

    Trajectory() : total_length(0) {}
    
    size_t size() const { return points.size(); }
    bool empty() const { return points.empty(); }
    void clear() { points.clear(); total_length = 0; }
};

/**
 * @enum FollowerStatus
 * @brief State of waypoint follower
 */
enum class FollowerStatus {
    IDLE = 0,
    PLANNING = 1,
    TRACKING = 2,
    GOAL_REACHED = 3,
    ERROR = 4
};

/**
 * @namespace math
 * @brief Math utility functions
 */
namespace math {
    /**
     * @brief Normalize angle
     */
    inline double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    /**
     * @brief Find the angle difference
     */
    inline double angle_diff(double a1, double a2) {
        return normalize_angle(a1 - a2);
    }

    /**
     * @brief Find the Euclidean Distance
     */
    inline double euclidean_distance(double x1, double y1, double x2, double y2) {
        return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }
    
    /**
     * @brief Wrap the angle in range [0, 2*pi]
     */
    inline double wrap_angle(double angle) {
        return normalize_angle(angle);
    }
}

}