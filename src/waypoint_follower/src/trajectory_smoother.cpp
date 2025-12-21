#include "waypoint_follower/trajectory_smoother.hpp"
#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

namespace waypoint_follower {

TrajectorySmoother::TrajectorySmoother(double max_curvature)
    : max_curvature_(max_curvature) {}

Trajectory
TrajectorySmoother::smooth_waypoints(const std::vector<Waypoint> &waypoints,
                                     double interpolation_distance) {

  Trajectory trajectory;

  if (waypoints.size() < 2) {
    RCLCPP_ERROR(rclcpp::get_logger("TrajectorySmoother"),
                 "Need at least 2 waypoints, got %zu", waypoints.size());
    return trajectory;
  }

  trajectory.points.clear();
  trajectory.total_length = 0.0;

  // Step 1: Convert waypoints to trajectory points
  for (const auto &wp : waypoints) {
    TrajectoryPoint tp;
    tp.x = wp.x;
    tp.y = wp.y;
    tp.theta = wp.theta;
    tp.velocity = wp.velocity;
    tp.curvature = 0.0;
    tp.s = 0.0;
    trajectory.points.push_back(tp);
  }

  // Step 2: Interpolate between waypoints
  Trajectory interpolated;

  for (size_t i = 0; i < trajectory.points.size() - 1; ++i) {
    // Add current point
    interpolated.points.push_back(trajectory.points[i]);

    // Calculate distance between consecutive waypoints
    double dx = trajectory.points[i + 1].x - trajectory.points[i].x;
    double dy = trajectory.points[i + 1].y - trajectory.points[i].y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < 1e-6)
      continue; // Skip if points are too close

    // Generate interpolation points
    int num_segments = static_cast<int>(dist / interpolation_distance) + 1;

    for (int j = 1; j < num_segments; ++j) {
      double ratio = static_cast<double>(j) / num_segments;

      TrajectoryPoint tp;
      tp.x = trajectory.points[i].x + ratio * dx;
      tp.y = trajectory.points[i].y + ratio * dy;

      // Interpolate heading linearly
      double theta_diff =
          trajectory.points[i + 1].theta - trajectory.points[i].theta;
      // Normalize angle difference to [-pi, pi]
      while (theta_diff > M_PI)
        theta_diff -= 2 * M_PI;
      while (theta_diff < -M_PI)
        theta_diff += 2 * M_PI;

      tp.theta = trajectory.points[i].theta + ratio * theta_diff;

      // Normalize theta to [-pi, pi]
      while (tp.theta > M_PI)
        tp.theta -= 2 * M_PI;
      while (tp.theta < -M_PI)
        tp.theta += 2 * M_PI;

      // Interpolate velocity
      tp.velocity = trajectory.points[i].velocity +
                    ratio * (trajectory.points[i + 1].velocity -
                             trajectory.points[i].velocity);

      tp.curvature = 0.0;
      tp.s = 0.0;

      interpolated.points.push_back(tp);
    }
  }

  // Add last point
  interpolated.points.push_back(trajectory.points.back());

  // Step 3: Compute arc lengths (cumulative distance along path)
  for (size_t i = 0; i < interpolated.points.size(); ++i) {
    if (i == 0) {
      interpolated.points[i].s = 0.0;
    } else {
      double dx = interpolated.points[i].x - interpolated.points[i - 1].x;
      double dy = interpolated.points[i].y - interpolated.points[i - 1].y;
      double ds = std::sqrt(dx * dx + dy * dy);
      interpolated.points[i].s = interpolated.points[i - 1].s + ds;
    }
  }

  interpolated.total_length = interpolated.points.back().s;

  // Step 4: Compute curvatures
  compute_curvatures(interpolated);

  // Step 5: Generate velocity profile
  generate_velocity_profile(interpolated);

  RCLCPP_INFO(rclcpp::get_logger("TrajectorySmoother"),
              "Generated trajectory with %zu points, total length: %.2f m",
              interpolated.points.size(), interpolated.total_length);

  return interpolated;
}

void TrajectorySmoother::generate_velocity_profile(Trajectory &trajectory) {
  if (trajectory.empty())
    return;

  // Create a velocity profile that accelerates and decelerates
  // Trapezoidal profile: ramp up, cruise, ramp down

  size_t num_points = trajectory.points.size();

  if (num_points < 3)
    return;

  // Find the point with the highest waypoint velocity (cruise velocity)
  double max_velocity = 0.0;
  for (const auto &point : trajectory.points) {
    max_velocity = std::max(max_velocity, point.velocity);
  }

  if (max_velocity < 1e-6)
    max_velocity = 0.5; // Default velocity

  // Create trapezoidal velocity profile
  // Accelerate for first 1/4 of path, cruise for middle 1/2, decelerate for
  // last 1/4

  size_t accel_end = num_points / 4;
  size_t decel_start = (3 * num_points) / 4;

  for (size_t i = 0; i < num_points; ++i) {
    if (i <= accel_end) {
      // Acceleration phase
      double ratio = static_cast<double>(i) / accel_end;
      trajectory.points[i].velocity = max_velocity * ratio;
    } else if (i >= decel_start) {
      // Deceleration phase
      double ratio =
          static_cast<double>(num_points - i) / (num_points - decel_start);
      trajectory.points[i].velocity = max_velocity * ratio;
    } else {
      // Cruise phase
      trajectory.points[i].velocity = max_velocity;
    }

    // Ensure minimum velocity for smooth motion
    trajectory.points[i].velocity =
        std::max(0.1, trajectory.points[i].velocity);
  }
}

void TrajectorySmoother::compute_curvatures(Trajectory &trajectory) {
  if (trajectory.points.size() < 3)
    return;

  for (size_t i = 1; i < trajectory.points.size() - 1; ++i) {
    // Compute curvature using heading change over arc length
    double dtheta =
        trajectory.points[i + 1].theta - trajectory.points[i - 1].theta;

    // Normalize angle difference to [-pi, pi]
    while (dtheta > M_PI)
      dtheta -= 2 * M_PI;
    while (dtheta < -M_PI)
      dtheta += 2 * M_PI;

    double ds = trajectory.points[i + 1].s - trajectory.points[i - 1].s;

    if (ds > 1e-6) {
      trajectory.points[i].curvature = dtheta / ds;
    } else {
      trajectory.points[i].curvature = 0.0;
    }

    // Limit curvature to maximum
    if (std::abs(trajectory.points[i].curvature) > max_curvature_) {
      trajectory.points[i].curvature =
          (trajectory.points[i].curvature > 0 ? 1 : -1) * max_curvature_;
    }
  }

  // Handle first and last points
  if (trajectory.points.size() > 1) {
    trajectory.points[0].curvature = trajectory.points[1].curvature;
    trajectory.points.back().curvature =
        trajectory.points[trajectory.points.size() - 2].curvature;
  }
}

std::vector<double> TrajectorySmoother::solve_tridiagonal(
    const std::vector<double> &a, const std::vector<double> &b,
    const std::vector<double> &c, const std::vector<double> &d) {

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
  // Placeholder for cubic spline interpolation
  return 0.0 * t; // Return dummy value
}

} // namespace waypoint_follower
