#pragma once

#include "waypoint_types.hpp"
#include <vector>

namespace waypoint_follower {

class TrajectorySmoother {
public:
    /**
     * @brief Constructor
     * @param max_curvature Maximum allowed curvature
     */
    explicit TrajectorySmoother(double max_curvature);

    /**
     * @brief Smooth waypoints into a trajectory
     * @param waypoints Input waypoints
     * @param interpolation_distance Distance between interpolated points
     * @return Smoothed trajectory
     */
    Trajectory smooth_waypoints(
        const std::vector<Waypoint>& waypoints,
        double interpolation_distance);

    /**
     * @brief Generate velocity profile along trajectory
     * @param trajectory Trajectory to modify
     */
    void generate_velocity_profile(Trajectory& trajectory);

    /**
     * @brief Compute curvatures for trajectory points
     * @param trajectory Trajectory to process
     */
    void compute_curvatures(Trajectory& trajectory);

    /**
     * @brief Solve tridiagonal system of equations
     * @param a Lower diagonal
     * @param b Main diagonal
     * @param c Upper diagonal
     * @param d Right-hand side
     * @return Solution vector
     */
    std::vector<double> solve_tridiagonal(
        const std::vector<double>& a,
        const std::vector<double>& b,
        const std::vector<double>& c,
        const std::vector<double>& d);

    /**
     * @brief Cubic spline interpolation
     * @param t Parameter value
     * @return Interpolated value
     */
    double cubic_spline_interpolate(double t);

private:
    double max_curvature_;
};

} // namespace waypoint_follower
