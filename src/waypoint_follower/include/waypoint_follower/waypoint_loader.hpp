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
  size_t size() const { return waypoints_.size(); }
  void clear() { waypoints_.clear(); }

  bool load_from_yaml(const std::string &yaml_file);
  bool load_from_csv(const std::string &csv_file);
  void add_waypoint(const Waypoint &wp);
  bool validate();

  void
  load_from_poses(const std::vector<geometry_msgs::msg::PoseStamped> &poses);

  // Inline accessors to avoid redefinition
  const std::vector<Waypoint> &get_waypoints() const { return waypoints_; }
};

} // namespace waypoint_follower
