/**
 * @file waypoint_loader.hpp
 * @brief Waypoint file loading and validation
 * @author Harsh Mulodhia
 * @version 1.0.0
 *
 * @class WaypointLoader
 * @brief Loads waypoints from external files and validates them
 *
 * Supports multiple input formats:
 * - YAML: Human-readable configuration format
 * - CSV: Comma-separated values for spreadsheet compatibility
 * - ROS 2 Poses: From action goals (geometry_msgs/PoseStamped)
 *
 * Features:
 * - Graceful error handling for malformed entries
 * - Auto-computation of heading angles when not specified
 * - Velocity validation and clamping
 * - Comprehensive logging at each stage
 *
 * @note All waypoints are validated after loading
 * @see validate() for validation algorithm details
 */

#pragma once

#include "waypoint_follower/waypoint_types.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <vector>

namespace waypoint_follower {

class WaypointLoader {
private:
  std::vector<Waypoint> waypoints_;

public:
  /// Query number of loaded waypoints
  size_t size() const { return waypoints_.size(); }

  /// Clear all waypoints
  void clear() { waypoints_.clear(); }
  
  /**
   * @brief waypoints from YAML file
   * @param yaml_file Path to YAML file with 'waypoints' key
   * @return true if loading successful, false otherwise
   * @note YAML format: 
   *   waypoints:
   *     - x: 1.0
   *       y: 2.0
   *       velocity: 0.5
   *       theta: 0.0  # optional
   */
  bool load_from_yaml(const std::string &yaml_file);

  /**
   * Load waypoints from CSV file
   * @param csv_file Path to CSV file (x, y, velocity, theta)
   * @return true if loading successful, false otherwise
   */
  bool load_from_csv(const std::string &csv_file);

  /**
   * Add single waypoint to collection
   * @param wp Waypoint to add
   */
  void add_waypoint(const Waypoint &wp);

  /**
   * Validate loaded waypoints
   * @return true if all waypoints valid
   * @details Checks: positive velocities, finite coordinates, normalizes angles
   *          Auto-computes heading to next waypoint if theta=0
   */
  bool validate();

  /**
   * Load waypoints from ROS 2 action goal (geometry_msgs::PoseStamped)
   * @param poses Vector of PoseStamped messages from action server
   */
  void load_from_poses(const std::vector<geometry_msgs::msg::PoseStamped> &poses);

  // Inline accessors to avoid redefinition
  const std::vector<Waypoint> &get_waypoints() const { return waypoints_; }
};

} // namespace waypoint_follower
