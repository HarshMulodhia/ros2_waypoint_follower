/**
 * @file test_trajectory_smoother.cpp
 * @brief Comprehensive unit tests for TrajectorySmoother
 * @author Autonomous Vehicle Team
 * @date December 2025
 * @version 2.0.1
 */

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include "waypoint_follower/trajectory_smoother.hpp"

using namespace waypoint_follower;

class TrajectorySmootherTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create smoother with max curvature constraint
        smoother_ = std::make_unique<TrajectorySmoother>(0.5);
    }
    
    std::unique_ptr<TrajectorySmoother> smoother_;
};

// ============================================================================
// Initialization Tests
// ============================================================================

TEST_F(TrajectorySmootherTest, SmootherInitialization) {
    EXPECT_NE(smoother_, nullptr);
}

// ============================================================================
// Simple Path Tests
// ============================================================================

TEST_F(TrajectorySmootherTest, SmoothSimpleLinearPath) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(Waypoint(0.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(5.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(10.0, 0.0, 0.5));
    
    Trajectory trajectory = smoother_->smooth_waypoints(waypoints, 0.1);
    
    EXPECT_GT(trajectory.size(), 3);
    EXPECT_GT(trajectory.total_length, 0.0);
    EXPECT_NEAR(trajectory.total_length, 10.0, 0.5);
}

TEST_F(TrajectorySmootherTest, SmoothTwoWaypointPath) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(Waypoint(0.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(10.0, 0.0, 0.5));
    
    Trajectory trajectory = smoother_->smooth_waypoints(waypoints, 0.5);
    
    EXPECT_GT(trajectory.size(), 2);
    EXPECT_NEAR(trajectory.total_length, 10.0, 0.5);
}

// ============================================================================
// Complex Path Tests
// ============================================================================

TEST_F(TrajectorySmootherTest, SmoothComplexWaypointPath) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(Waypoint(0.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(5.0, 5.0, 0.5));
    waypoints.push_back(Waypoint(10.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(5.0, -5.0, 0.5));
    
    Trajectory trajectory = smoother_->smooth_waypoints(waypoints, 0.1);
    
    EXPECT_GT(trajectory.size(), 10);
    EXPECT_GT(trajectory.total_length, 0.0);
}

TEST_F(TrajectorySmootherTest, SmoothRectangularPath) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(Waypoint(0.0, 0.0, 0.5));   // Start
    waypoints.push_back(Waypoint(10.0, 0.0, 0.5));  // Right
    waypoints.push_back(Waypoint(10.0, 5.0, 0.5));  // Up
    waypoints.push_back(Waypoint(0.0, 5.0, 0.5));   // Left
    waypoints.push_back(Waypoint(0.0, 0.0, 0.5));   // Return
    
    Trajectory trajectory = smoother_->smooth_waypoints(waypoints, 0.1);
    
    EXPECT_GT(trajectory.size(), 20);
    // Rectangular path ≈ 30m perimeter
    EXPECT_NEAR(trajectory.total_length, 30.0, 1.0);
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST_F(TrajectorySmootherTest, SingleWaypointPath) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(Waypoint(0.0, 0.0, 0.5));
    
    // Single waypoint should not produce trajectory
    Trajectory trajectory = smoother_->smooth_waypoints(waypoints, 0.1);
    
    EXPECT_LE(trajectory.size(), 1);
}

TEST_F(TrajectorySmootherTest, EmptyWaypointPath) {
    std::vector<Waypoint> waypoints;
    
    Trajectory trajectory = smoother_->smooth_waypoints(waypoints, 0.1);
    
    EXPECT_EQ(trajectory.size(), 0);
    EXPECT_EQ(trajectory.total_length, 0.0);
    EXPECT_TRUE(trajectory.empty());
}

TEST_F(TrajectorySmootherTest, IdenticalConsecutiveWaypoints) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(Waypoint(0.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(0.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(10.0, 0.0, 0.5));
    
    // Should handle duplicate waypoints gracefully
    EXPECT_NO_THROW(smoother_->smooth_waypoints(waypoints, 0.1));
}

// ============================================================================
// Velocity Profile Tests
// ============================================================================

TEST_F(TrajectorySmootherTest, GenerateVelocityProfile) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(Waypoint(0.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(5.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(10.0, 0.0, 0.5));
    
    Trajectory trajectory = smoother_->smooth_waypoints(waypoints, 0.1);
    smoother_->generate_velocity_profile(trajectory);
    
    // All velocities should be non-negative
    for (const auto& point : trajectory.points) {
        EXPECT_GE(point.velocity, 0.0);
    }
}

TEST_F(TrajectorySmootherTest, VelocityProfileVaryingPath) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(Waypoint(0.0, 0.0, 1.0));   // Fast
    waypoints.push_back(Waypoint(5.0, 5.0, 0.3));   // Slow (turn)
    waypoints.push_back(Waypoint(10.0, 0.0, 1.0));  // Fast
    
    Trajectory trajectory = smoother_->smooth_waypoints(waypoints, 0.1);
    smoother_->generate_velocity_profile(trajectory);
    
    // Should have valid velocities
    EXPECT_GT(trajectory.points.size(), 0);
    for (const auto& point : trajectory.points) {
        EXPECT_TRUE(std::isfinite(point.velocity));
        EXPECT_GE(point.velocity, 0.0);
    }
}

// ============================================================================
// Curvature Computation Tests
// ============================================================================

TEST_F(TrajectorySmootherTest, ComputeCurvatures) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(Waypoint(0.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(2.5, 2.5, 0.5));
    waypoints.push_back(Waypoint(5.0, 0.0, 0.5));
    
    Trajectory trajectory = smoother_->smooth_waypoints(waypoints, 0.1);
    smoother_->compute_curvatures(trajectory);
    
    // Should have curvature for curved path
    bool has_non_zero_curvature = false;
    for (const auto& point : trajectory.points) {
        EXPECT_TRUE(std::isfinite(point.curvature));
        EXPECT_LE(std::abs(point.curvature), 0.5);  // Respects max curvature
        if (std::abs(point.curvature) > 1e-6) {
            has_non_zero_curvature = true;
        }
    }
    EXPECT_TRUE(has_non_zero_curvature);
}

TEST_F(TrajectorySmootherTest, ComputeCurvaturesLinearPath) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(Waypoint(0.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(5.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(10.0, 0.0, 0.5));
    
    Trajectory trajectory = smoother_->smooth_waypoints(waypoints, 0.1);
    smoother_->compute_curvatures(trajectory);
    
    // Linear path should have near-zero curvature
    for (const auto& point : trajectory.points) {
        EXPECT_NEAR(point.curvature, 0.0, 0.01);
    }
}

// ============================================================================
// Interpolation Parameter Tests
// ============================================================================

TEST_F(TrajectorySmootherTest, DifferentInterpolationDistances) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(Waypoint(0.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(10.0, 0.0, 0.5));
    
    // Fine interpolation
    Trajectory traj_fine = smoother_->smooth_waypoints(waypoints, 0.05);
    
    // Coarse interpolation
    Trajectory traj_coarse = smoother_->smooth_waypoints(waypoints, 0.5);
    
    // Fine should have more points
    EXPECT_GT(traj_fine.size(), traj_coarse.size());
    
    // Both should have same length
    EXPECT_NEAR(traj_fine.total_length, traj_coarse.total_length, 0.1);
}

// ============================================================================
// Arc Length Tests
// ============================================================================

TEST_F(TrajectorySmootherTest, ArcLengthParameterization) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(Waypoint(0.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(5.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(10.0, 0.0, 0.5));
    
    Trajectory trajectory = smoother_->smooth_waypoints(waypoints, 0.1);
    
    // Arc length should be monotonically increasing
    double prev_s = 0.0;
    for (const auto& point : trajectory.points) {
        EXPECT_GE(point.s, prev_s);
        prev_s = point.s;
    }
    
    // Final s should equal total_length
    if (!trajectory.points.empty()) {
        EXPECT_NEAR(trajectory.points.back().s, trajectory.total_length, 0.01);
    }
}

// ============================================================================
// Continuity Tests
// ============================================================================

TEST_F(TrajectorySmootherTest, PositionContinuity) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(Waypoint(0.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(5.0, 5.0, 0.5));
    waypoints.push_back(Waypoint(10.0, 0.0, 0.5));
    
    Trajectory trajectory = smoother_->smooth_waypoints(waypoints, 0.05);
    
    // Check no sudden jumps in position
    for (size_t i = 1; i < trajectory.points.size(); ++i) {
        double dist = std::hypot(
            trajectory.points[i].x - trajectory.points[i-1].x,
            trajectory.points[i].y - trajectory.points[i-1].y);
        
        // Gap should not exceed interpolation distance + margin
        EXPECT_LT(dist, 0.2);
    }
}

TEST_F(TrajectorySmootherTest, HeadingContinuity) {
    std::vector<Waypoint> waypoints;
    waypoints.push_back(Waypoint(0.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(5.0, 0.0, 0.5));
    waypoints.push_back(Waypoint(10.0, 0.0, 0.5));
    
    Trajectory trajectory = smoother_->smooth_waypoints(waypoints, 0.1);
    
    // Check smooth heading changes
    for (size_t i = 1; i < trajectory.points.size(); ++i) {
        double heading_diff = std::abs(
            trajectory.points[i].theta - trajectory.points[i-1].theta);
        
        // Heading change per step should be small
        EXPECT_LT(heading_diff, 0.2);
    }
}

// ============================================================================
// Performance Tests
// ============================================================================

TEST_F(TrajectorySmootherTest, LargeWaypointSet) {
    std::vector<Waypoint> waypoints;
    
    // Create a spiral path with 100 waypoints
    for (int i = 0; i < 100; ++i) {
        double angle = i * 0.1;
        double radius = 0.1 * i;
        double x = radius * std::cos(angle);
        double y = radius * std::sin(angle);
        waypoints.push_back(Waypoint(x, y, 0.5));
    }
    
    Trajectory trajectory = smoother_->smooth_waypoints(waypoints, 0.1);
    
    EXPECT_GT(trajectory.size(), 100);
    EXPECT_GT(trajectory.total_length, 0.0);
}

TEST_F(TrajectorySmootherTest, FineInterpolationPerformance) {
    std::vector<Waypoint> waypoints;
    for (int i = 0; i < 20; ++i) {
        waypoints.push_back(Waypoint(i * 1.0, 0.0, 0.5));
    }
    
    // Should handle fine interpolation efficiently
    auto start = std::chrono::high_resolution_clock::now();
    Trajectory trajectory = smoother_->smooth_waypoints(waypoints, 0.01);
    auto end = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    // Should complete in reasonable time (<100ms)
    EXPECT_LT(duration.count(), 100);
    EXPECT_GT(trajectory.size(), 0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
