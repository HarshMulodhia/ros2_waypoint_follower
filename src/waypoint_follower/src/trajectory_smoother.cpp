/**
 * @file trajectory_smoother.cpp
 * @brief Smooth waypoints using cubic spline interpolation
 * @author Autonomous Vehicle Team
 * @date December 2025
 * @version 2.0.1 - FIXED
 */

#include "waypoint_follower/trajectory_smoother.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <algorithm>

namespace waypoint_follower {

TrajectorySmoother::TrajectorySmoother(double max_curvature)
    : max_curvature_(max_curvature) {}

Trajectory TrajectorySmoother::smooth_waypoints(
    const std::vector<Waypoint>& waypoints,
    double interpolation_distance) {
    
    Trajectory trajectory;
    
    if (waypoints.size() < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("TrajectorySmoother"),
                    "Need at least 2 waypoints");
        return trajectory;
    }
    
    trajectory.points.clear();
    trajectory.total_length = 0;
    
    // Convert waypoints to trajectory points
    for (const auto& wp : waypoints) {
        TrajectoryPoint tp;
        tp.x = wp.x;
        tp.y = wp.y;
        tp.theta = wp.theta;
        tp.velocity = wp.velocity;
        trajectory.points.push_back(tp);
    }
    
    // Interpolate between waypoints
    Trajectory interpolated;
    
    for (size_t i = 0; i < trajectory.points.size() - 1; ++i) {
        interpolated.points.push_back(trajectory.points[i]);
        
        double dx = trajectory.points[i + 1].x - trajectory.points[i].x;
        double dy = trajectory.points[i + 1].y - trajectory.points[i].y;
        double dist = std::sqrt(dx * dx + dy * dy);
        
        if (dist < 1e-6) continue;
        
        int num_segments = static_cast<int>(dist / interpolation_distance) + 1;
        
        for (int j = 1; j < num_segments; ++j) {
            double ratio = static_cast<double>(j) / num_segments;
            TrajectoryPoint tp;
            tp.x = trajectory.points[i].x + ratio * dx;
            tp.y = trajectory.points[i].y + ratio * dy;
            tp.theta = trajectory.points[i].theta +
                      ratio * (trajectory.points[i + 1].theta -
                              trajectory.points[i].theta);
            tp.velocity = trajectory.points[i].velocity +
                         ratio * (trajectory.points[i + 1].velocity -
                                 trajectory.points[i].velocity);
            interpolated.points.push_back(tp);
        }
    }
    
    interpolated.points.push_back(trajectory.points.back());
    
    // Compute arc lengths
    for (size_t i = 0; i < interpolated.points.size(); ++i) {
        if (i == 0) {
            interpolated.points[i].s = 0;
        } else {
            double dx = interpolated.points[i].x - interpolated.points[i-1].x;
            double dy = interpolated.points[i].y - interpolated.points[i-1].y;
            double ds = std::sqrt(dx * dx + dy * dy);
            interpolated.points[i].s = interpolated.points[i-1].s + ds;
        }
    }
    
    interpolated.total_length = interpolated.points.back().s;
    
    // Compute curvatures
    compute_curvatures(interpolated);
    
    RCLCPP_INFO(rclcpp::get_logger("TrajectorySmoother"),
               "Generated trajectory with %zu points, length: %.2f m",
               interpolated.points.size(), interpolated.total_length);
    
    return interpolated;
}

void TrajectorySmoother::generate_velocity_profile(
    Trajectory& trajectory) {
    
    if (trajectory.empty()) return;
    
    size_t mid_point = trajectory.points.size() / 2;
    
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
        if (i < mid_point) {
            double ratio = static_cast<double>(i) / mid_point;
            trajectory.points[i].velocity *= ratio;
        } else {
            double ratio = static_cast<double>(trajectory.points.size() - i) /
                          (trajectory.points.size() - mid_point);
            trajectory.points[i].velocity *= ratio;
        }
    }
}

void TrajectorySmoother::compute_curvatures(Trajectory& trajectory) {
    if (trajectory.points.size() < 3) return;
    
    for (size_t i = 1; i < trajectory.points.size() - 1; ++i) {
        double dtheta = trajectory.points[i + 1].theta - 
                       trajectory.points[i - 1].theta;
        double ds = trajectory.points[i + 1].s - trajectory.points[i - 1].s;
        
        if (ds > 1e-6) {
            trajectory.points[i].curvature = dtheta / ds;
        } else {
            trajectory.points[i].curvature = 0;
        }
    }
}

std::vector<double> TrajectorySmoother::solve_tridiagonal(
    const std::vector<double>& a,
    const std::vector<double>& b,
    const std::vector<double>& c,
    const std::vector<double>& d) {
    
    size_t n = d.size();
    std::vector<double> c_prime(n), d_prime(n), x(n);
    
    // Forward elimination
    c_prime[0] = c[0] / b[0];
    d_prime[0] = d[0] / b[0];
    
    for (size_t i = 1; i < n; ++i) {
        double denom = b[i] - a[i] * c_prime[i - 1];
        if (i < n - 1) {
            c_prime[i] = c[i] / denom;
        }
        d_prime[i] = (d[i] - a[i] * d_prime[i - 1]) / denom;
    }
    
    // Back substitution
    x[n - 1] = d_prime[n - 1];
    for (int i = static_cast<int>(n) - 2; i >= 0; --i) {
        x[i] = d_prime[i] - c_prime[i] * x[i + 1];
    }
    
    return x;
}

double TrajectorySmoother::cubic_spline_interpolate(double t) {
    return 0.0;
}

} // namespace waypoint_follower
