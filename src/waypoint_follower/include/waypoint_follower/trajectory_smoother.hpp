#pragma once

#include "waypoint_types.hpp"
#include <vector>

namespace waypoint_follower {

/**
 * @class TrajectorySmoother
 * @brief Smooth waypoints using cubic spline interpolation
 */
class TrajectorySmoother {
public:
    /// Constructor
    TrajectorySmoother(double max_curvature = 0.5);
    ~TrajectorySmoother() = default;

    /// Smooth waypoints into trajectory
    Trajectory smooth_waypoints(const std::vector<Waypoint>& waypoints,
                               double interpolation_distance = 0.1);

    /// Generate velocity profile
    void generate_velocity_profile(Trajectory& trajectory,
                                  double max_acceleration = 0.5);

    /// Compute curvatures
    void compute_curvatures(Trajectory& trajectory);

private:
    double max_curvature_;

    std::vector<double> solve_tridiagonal(const std::vector<double>& a,
                                         const std::vector<double>& b,
                                         const std::vector<double>& c,
                                         const std::vector<double>& d);

    double cubic_spline_interpolate(const std::vector<Waypoint>& waypoints,
                                   const std::vector<std::vector<double>>& coeffs,
                                   double t, bool interpolate_y = false);
};

}
