#pragma once

#include "waypoint_types.hpp"
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace waypoint_follower {

/**
 * @class WaypointLoader
 * @brief Load waypoints from YAML or CSV files
 */
class WaypointLoader {
public:
    WaypointLoader() = default;
    ~WaypointLoader() = default;

    /// Load waypoints from YAML file
    bool load_from_yaml(const std::string& yaml_file);

    /// Load waypoints from CSV file
    bool load_from_csv(const std::string& csv_file);

    /// Add single waypoint
    void add_waypoint(const Waypoint& wp);

    /// Validate waypoints
    bool validate();

    /// Get all waypoints
    const std::vector<Waypoint>& get_waypoints() const { return waypoints_; }

    /// Get waypoint count
    size_t size() const { return waypoints_.size(); }

    /// Clear all waypoints
    void clear() { waypoints_.clear(); }

private:
    std::vector<Waypoint> waypoints_;
};

}  // namespace waypoint_follower
