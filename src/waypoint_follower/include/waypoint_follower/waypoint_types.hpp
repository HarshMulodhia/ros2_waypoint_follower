/**
 * @file waypoint_types.hpp
 * @brief Data structures for waypoint following system
 * @author Harsh Mulodhia
 * @version 1.0.0
 *
 * Defines core data types used throughout the waypoint follower system:
 * - Waypoint: Single navigation target
 * - RobotState: Complete state estimate (6-DOF)
 * - ControlCommand: Velocity command output
 * - TrackingError: Path tracking error metrics
 * - Trajectory: Collection of smoothed waypoints
 * - FollowerStatus: State machine states
 * - Math utilities: Angle normalization, distance calculation
 */

#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <vector>

namespace waypoint_follower {

/**
 * @struct Waypoint
 * @brief Single waypoint with position, orientation, and velocity
 * @details Represents a target waypoint in 2D space with desired heading and velocity
 */
struct Waypoint {
  double x;        ///< X position (meters)
  double y;        ///< Y position (meters)
  double theta;    ///< Desired heading (radians)
  double velocity; ///< Desired velocity (m/s)

  /// Default constructor: initializes all fields to zero
  Waypoint() : x(0), y(0), theta(0), velocity(0.5) {}

  /// Parametric constructor for convenient initialization
  Waypoint(double x_, double y_, double v_ = 0.5, double t_ = 0)
      : x(x_), y(y_), theta(t_), velocity(v_) {}
};

/**
 * @struct RobotState
 * @brief Complete 6-DOF robot state estimate
 * @details Maintains full kinematic state: [x, y, θ, vx, vy, ω]
 * @note Used by Kalman filter output and controller input
 */
struct RobotState {
  double x;         ///< X position (meters)
  double y;         ///< Y position (meters)
  double theta;     ///< Heading angle (radians)
  double vx;        ///< X velocity (m/s)
  double vy;        ///< Y velocity (m/s)
  double omega;     ///< Angular velocity (rad/s)
  double timestamp; ///< State timestamp (seconds since epoch)

  /// Default constructor: initializes all fields to zero
  RobotState() : x(0), y(0), theta(0), vx(0), vy(0), omega(0), timestamp(0) {}

  /// Accessor for position vector
  Eigen::Vector3d position() const { return Eigen::Vector3d(x, y, theta); }

  /// Accessor for velocity vector
  Eigen::Vector3d velocity() const { return Eigen::Vector3d(vx, vy, omega); }
};

/**
 * @struct ControlCommand
 * @brief Velocity command for differential-drive robot
 * @details Output of path tracking controller
 */

struct ControlCommand {
  double linear_velocity;  ///< Forward velocity (m/s)
  double lateral_velocity; ///< Lateral velocity (m/s)
  double angular_velocity; ///< Angular velocity (rad/s)

  ControlCommand()
      : linear_velocity(0), lateral_velocity(0), angular_velocity(0) {}

  ControlCommand(double lin, double lat, double ang)
      : linear_velocity(lin), lateral_velocity(lat), angular_velocity(ang) {}
};

/**
 * @struct TrackingError
 * @brief Path tracking performance metrics
 * @details Computed by control loop for feedback to path tracker
 */
struct TrackingError {
  double cross_track_error; ///< Lateral deviation from path (meters)
  double heading_error;     ///< Heading mismatch (radians, [-π, π])
  double velocity_error;    ///< Velocity mismatch (m/s)
  double progress;          ///< Progress along trajectory [0.0, 1.0]

  TrackingError()
      : cross_track_error(0), heading_error(0), velocity_error(0), progress(0) {
      }
};

/**
 * @struct TrajectoryPoint
 * @brief Single interpolated trajectory point
 * @details Contains position, orientation, curvature, and velocity at waypoint
 */
struct TrajectoryPoint {
  double x;           ///< X position (meters)
  double y;           ///< Y position (meters)
  double theta;       ///< Path heading (radians)
  double curvature;   ///< Instantaneous path curvature (1/meters)
  double velocity;    ///< Desired velocity profile (m/s)
  double s;           ///< Arc length from trajectory start (meters)

  TrajectoryPoint() : x(0), y(0), theta(0), curvature(0), velocity(0.5), s(0) {}
};

/**
 * @struct Trajectory
 * @brief Complete smoothed trajectory
 * @details Vector of interpolated trajectory points with metadata
 */
struct Trajectory {
  std::vector<TrajectoryPoint> points; ///< Ordered waypoints along path
  double total_length;                 ///< Total path length (meters)

  Trajectory() : total_length(0) {}

  size_t size() const { return points.size(); }
  bool empty() const { return points.empty(); }
  void clear() {
    points.clear();
    total_length = 0;
  }
};

/**
 * @enum FollowerStatus
 * @brief State of waypoint follower
 */
enum class FollowerStatus {
  IDLE = 0,         ///< Waiting for goal
  PLANNING = 1,     ///< Generating trajectory
  TRACKING = 2,     ///< Following path
  GOAL_REACHED = 3, ///< Destination reached
  ERROR = 4         ///< Error state
};

/**
 * @namespace math
 * @brief Math utility functions
 */
namespace math {

/**
 * @brief Normalize angle to [-π, π]
 * @param angle Input angle in radians
 * @return Normalized angle in [-π, π]
 */
inline double normalize_angle(double angle) {
  while (angle > M_PI)
    angle -= 2 * M_PI;
  while (angle < -M_PI)
    angle += 2 * M_PI;
  return angle;
}

/**
 * @brief Calculate angle difference
 * @param a1 First angle in radians
 * @param a2 Second angle in radians
 * @return Angle difference normalized to [-π, π]
 */
inline double angle_diff(double a1, double a2) {
  return normalize_angle(a1 - a2);
}

/**
 * @brief Calculate Euclidean distance
 * @param x1 First point X coordinate
 * @param y1 First point Y coordinate
 * @param x2 Second point X coordinate
 * @param y2 Second point Y coordinate
 * @return Distance in meters
 */
inline double euclidean_distance(double x1, double y1, double x2, double y2) {
  return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

/**
 * @brief Wrap angle to [0, 2π]
 * @param angle Input angle in radians
 * @return Wrapped angle in [0, 2π]
 */
inline double wrap_angle(double angle) { return normalize_angle(angle); }

/**
 * @brief Check if value is valid (not NaN or Inf)
 * @param val Value to check
 * @return True if valid, false otherwise
 */
inline bool is_valid(double val) { return std::isfinite(val); }

/**
 * @brief Clamp value to range [min, max]
 * @param val Value to clamp
 * @param min Minimum value
 * @param max Maximum value
 * @return Clamped value
 */
inline double clamp(double val, double min, double max) {
  return std::max(min, std::min(val, max));
}

} // namespace math

} // namespace waypoint_follower
