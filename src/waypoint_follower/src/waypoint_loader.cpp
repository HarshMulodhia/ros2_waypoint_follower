#include "waypoint_follower/waypoint_loader.hpp"
#include <fstream>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

namespace waypoint_follower {

bool WaypointLoader::load_from_yaml(const std::string& yaml_file) {
    try {
        YAML::Node config = YAML::LoadFile(yaml_file);
        if (!config["waypoints"]) {
            RCLCPP_ERROR(rclcpp::get_logger("WaypointLoader"), "No waypoints in YAML");
            return false;
        }
        waypoints_.clear();
        for (const auto& wp_node : config["waypoints"]) {
            Waypoint wp;
            wp.x = wp_node["x"].as<double>();
            wp.y = wp_node["y"].as<double>();
            wp.velocity = wp_node["velocity"].as<double>(0.5);
            wp.theta = wp_node["theta"].as<double>(0.0);
            waypoints_.push_back(wp);
        }
        RCLCPP_INFO(rclcpp::get_logger("WaypointLoader"),
                   "Loaded %zu waypoints from YAML", waypoints_.size());
        return validate();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("WaypointLoader"), "Error: %s", e.what());
        return false;
    }
}

bool WaypointLoader::load_from_csv(const std::string& csv_file) {
    try {
        std::ifstream file(csv_file);
        if (!file.is_open()) {
            RCLCPP_ERROR(rclcpp::get_logger("WaypointLoader"), "Cannot open CSV");
            return false;
        }
        waypoints_.clear();
        std::string line;
        std::getline(file, line);  // Skip header
        while (std::getline(file, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            double x, y, velocity = 0.5, theta = 0.0;
            char comma;
            ss >> x >> comma >> y >> comma >> velocity >> comma >> theta;
            if (ss.fail()) continue;
            waypoints_.emplace_back(x, y, velocity, theta);
        }
        file.close();
        RCLCPP_INFO(rclcpp::get_logger("WaypointLoader"),
                   "Loaded %zu waypoints from CSV", waypoints_.size());
        return validate();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("WaypointLoader"), "Error: %s", e.what());
        return false;
    }
}

void WaypointLoader::add_waypoint(const Waypoint& wp) {
    waypoints_.push_back(wp);
}

bool WaypointLoader::validate() {
    if (waypoints_.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("WaypointLoader"), "No waypoints");
        return false;
    }
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        if (waypoints_[i].velocity <= 0) {
            RCLCPP_WARN(rclcpp::get_logger("WaypointLoader"),
                       "Invalid velocity at %zu", i);
            waypoints_[i].velocity = 0.5;
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("WaypointLoader"),
               "Validation passed for %zu waypoints", waypoints_.size());
    return true;
}

}  // namespace waypoint_follower
