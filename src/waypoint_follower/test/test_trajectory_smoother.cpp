#include <gtest/gtest.h>
#include "waypoint_follower/trajectory_smoother.hpp"
#include <cmath>

using namespace waypoint_follower;

class TrajectorySmootherTest : public ::testing::Test {
protected:
    void SetUp() override {
        smoother = std::make_unique<TrajectorySmoother>(0.5);
    }

    std::unique_ptr<TrajectorySmoother> smoother;
};

TEST_F(TrajectorySmootherTest, SmootherInitialization) {
    EXPECT_NE(smoother, nullptr);
}

TEST_F(TrajectorySmootherTest, SmoothSimplePath) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(Waypoint(0.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(5.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(10.0, 0.0, 0.5));

    Trajectory trajectory = smoother->smooth_waypoints(waypoints, 0.1);
    
    EXPECT_GT(trajectory.size(), 3);
    EXPECT_GT(trajectory.total_length, 0.0);
}

TEST_F(TrajectorySmootherTest, SmoothComplexPath) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(Waypoint(0.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(5.0, 5.0, 0.5));
    waypoints.push_back(Waypoint(10.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(0.0, 0.0, 0.5));

    Trajectory trajectory = smoother->smooth_waypoints(waypoints, 0.1);
    
    EXPECT_GT(trajectory.size(), 10);
    EXPECT_GT(trajectory.total_length, 0.0);
}

TEST_F(TrajectorySmootherTest, VelocityProfile) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(Waypoint(0.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(5.0, 0.0, 0.5));

    Trajectory trajectory = smoother->smooth_waypoints(waypoints);
    smoother->generate_velocity_profile(trajectory);

    for (const auto& point : trajectory.points) {
        EXPECT_GE(point.velocity, 0.0);
    }
}

TEST_F(TrajectorySmootherTest, ComputeCurvatures) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(Waypoint(0.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(2.5, 2.5, 0.5));
    waypoints.push_back(Waypoint(5.0, 0.0, 0.5));

    Trajectory trajectory = smoother->smooth_waypoints(waypoints);
    smoother->compute_curvatures(trajectory);

    bool has_non_zero_curvature = false;
    for (const auto& point : trajectory.points) {
        if (std::abs(point.curvature) > 1e-6) {
            has_non_zero_curvature = true;
            break;
        }
    }
    EXPECT_TRUE(has_non_zero_curvature);
}

TEST_F(TrajectorySmootherTest, SingleWaypointPath) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(Waypoint(0.0, 0.0, 0.5));

    Trajectory trajectory = smoother->smooth_waypoints(waypoints);
    EXPECT_EQ(trajectory.size(), 0);
}

TEST_F(TrajectorySmootherTest, TwoWaypointPath) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(Waypoint(0.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(10.0, 0.0, 0.5));

    Trajectory trajectory = smoother->smooth_waypoints(waypoints, 0.5);
    
    EXPECT_GT(trajectory.size(), 2);
    EXPECT_NEAR(trajectory.total_length, 10.0, 0.1);
}
