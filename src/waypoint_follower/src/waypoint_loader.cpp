/**
 * @file waypoint_loader.cpp
 * @brief Waypoint loader implementation
 * @author Autonomous Vehicle Team
 * @date December 2025
 * @version 2.0.1 - FIXED
 */

#include "waypoint_follower/waypoint_loader.hpp"
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>
#include <cmath>

namespace waypoint_follower {

bool WaypointLoader::load_from_yaml(const std::string& yaml_file) {
    try {
        RCLCPP_INFO(rclcpp::get_logger("WaypointLoader"), 
                   "Loading waypoints from YAML file: %s", yaml_file.c_str());
        
        YAML::Node config = YAML::LoadFile(yaml_file);
        
        if (!config["waypoints"]) {
            RCLCPP_ERROR(rclcpp::get_logger("WaypointLoader"), 
                        "No 'waypoints' key found in YAML file");
            return false;
        }
        
        waypoints_.clear();
        
        for (const auto& wp_node : config["waypoints"]) {
            if (!wp_node["x"] || !wp_node["y"]) {
                RCLCPP_WARN(rclcpp::get_logger("WaypointLoader"), 
                           "Waypoint missing x or y coordinate, skipping");
                continue;
            }
            
            Waypoint wp;
            wp.x = wp_node["x"].as<double>();
            wp.y = wp_node["y"].as<double>();
            wp.velocity = wp_node["velocity"].as<double>(0.5);
            wp.theta = wp_node["theta"].as<double>(0.0);
            
            waypoints_.push_back(wp);
        }
        
        RCLCPP_INFO(rclcpp::get_logger("WaypointLoader"),
                   "Successfully loaded %zu waypoints from YAML", 
                   waypoints_.size());
        
        return validate();
        
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("WaypointLoader"), 
                    "YAML parsing error: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("WaypointLoader"), 
                    "Error loading YAML file: %s", e.what());
        return false;
    }
}

bool WaypointLoader::load_from_csv(const std::string& csv_file) {
    try {
        RCLCPP_INFO(rclcpp::get_logger("WaypointLoader"), 
                   "Loading waypoints from CSV file: %s", csv_file.c_str());
        
        std::ifstream file(csv_file);
        if (!file.is_open()) {
            RCLCPP_ERROR(rclcpp::get_logger("WaypointLoader"), 
                        "Cannot open CSV file: %s", csv_file.c_str());
            return false;
        }
        
        waypoints_.clear();
        std::string line;
        int line_number = 0;
        
        // Skip header
        if (!std::getline(file, line)) {
            RCLCPP_ERROR(rclcpp::get_logger("WaypointLoader"), 
                        "CSV file is empty");
            file.close();
            return false;
        }
        
        while (std::getline(file, line)) {
            ++line_number;
            
            if (line.empty() || line.find_first_not_of(" \t\r\n") == std::string::npos) {
                continue;
            }
            
            std::stringstream ss(line);
            double x, y, velocity = 0.5, theta = 0.0;
            char comma;
            
            if (!(ss >> x >> comma >> y >> comma >> velocity >> comma >> theta)) {
                RCLCPP_WARN(rclcpp::get_logger("WaypointLoader"), 
                           "Failed to parse CSV line %d, skipping", line_number);
                continue;
            }
            
            waypoints_.emplace_back(x, y, velocity, theta);
        }
        
        file.close();
        
        RCLCPP_INFO(rclcpp::get_logger("WaypointLoader"),
                   "Successfully loaded %zu waypoints from CSV", 
                   waypoints_.size());
        
        return validate();
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("WaypointLoader"), 
                    "Error loading CSV file: %s", e.what());
        return false;
    }
}

void WaypointLoader::add_waypoint(const Waypoint& wp) {
    waypoints_.push_back(wp);
    RCLCPP_DEBUG(rclcpp::get_logger("WaypointLoader"), 
                "Added waypoint at (%.2f, %.2f)", wp.x, wp.y);
}

bool WaypointLoader::validate() {
    if (waypoints_.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("WaypointLoader"), 
                    "Waypoint list is empty");
        return false;
    }
    
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        if (waypoints_[i].velocity <= 0) {
            RCLCPP_WARN(rclcpp::get_logger("WaypointLoader"),
                       "Invalid velocity (%.2f) at waypoint %zu, setting to 0.5 m/s", 
                       waypoints_[i].velocity, i);
            waypoints_[i].velocity = 0.5;
        }
        
        if (!std::isfinite(waypoints_[i].x) || 
            !std::isfinite(waypoints_[i].y) ||
            !std::isfinite(waypoints_[i].theta)) {
            RCLCPP_WARN(rclcpp::get_logger("WaypointLoader"),
                       "Waypoint %zu contains invalid coordinates, skipping", i);
            waypoints_.erase(waypoints_.begin() + i);
            --i;
        }
    }
    
    if (waypoints_.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("WaypointLoader"), 
                    "No valid waypoints after validation");
        return false;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("WaypointLoader"),
               "Validation completed for %zu waypoints", waypoints_.size());
    
    return true;
}

} // namespace waypoint_follower
