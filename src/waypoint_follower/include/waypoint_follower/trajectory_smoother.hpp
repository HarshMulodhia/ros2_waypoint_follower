#pragma once

#include "waypoint_follower/waypoint_types.hpp"
#include <vector>

namespace waypoint_follower {

class TrajectorySmoother {
private:
  double max_curvature_;

public:
  TrajectorySmoother(double max_curvature);

  Trajectory smooth_waypoints(const std::vector<Waypoint> &waypoints,
                              double interpolation_distance);

  void compute_curvatures(Trajectory &trajectory);
  void generate_velocity_profile(Trajectory &trajectory);
  std::vector<double> solve_tridiagonal(const std::vector<double> &a,
                                        const std::vector<double> &b,
                                        const std::vector<double> &c,
                                        const std::vector<double> &d);
  double cubic_spline_interpolate(double t);
};

} // namespace waypoint_follower
