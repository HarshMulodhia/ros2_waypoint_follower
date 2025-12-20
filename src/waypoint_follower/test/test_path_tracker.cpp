/**
 * @file test_path_tracker.cpp
 * @brief Unit tests for PathTrackerController
 * @author Autonomous Vehicle Team
 * @date December 2025
 * @version 2.0.1
 */

#include <gtest/gtest.h>
#include <cmath>
#include "waypoint_follower/path_tracker_controller.hpp"

using namespace waypoint_follower;

class PathTrackerControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create controller: dt=0.02s, max_steering=0.5rad, max_velocity=1.5m/s
        controller_ = std::make_unique<PathTrackerController>(0.02, 0.5, 1.5);
        
        // Initialize robot state
        robot_state_.x = 0.0;
        robot_state_.y = 0.0;
        robot_state_.theta = 0.0;
        robot_state_.vx = 0.5;
        robot_state_.vy = 0.0;
        robot_state_.omega = 0.0;
    }
    
    std::unique_ptr<PathTrackerController> controller_;
    RobotState robot_state_;
};

// ============================================================================
// Initialization Tests
// ============================================================================

TEST_F(PathTrackerControllerTest, ControllerInitialization) {
    EXPECT_NE(controller_, nullptr);
}

TEST_F(PathTrackerControllerTest, ControllerReset) {
    // Generate some control
    TrackingError error;
    error.cross_track_error = 0.1;
    error.heading_error = 0.05;
    error.velocity_error = 0.2;
    
    controller_->update(robot_state_, error);
    controller_->reset();
    
    // After reset, controller should start fresh
    error.cross_track_error = 0.0;
    error.heading_error = 0.0;
    error.velocity_error = 0.0;
    
    ControlCommand cmd = controller_->update(robot_state_, error);
    EXPECT_DOUBLE_EQ(cmd.angular_velocity, 0.0);
    EXPECT_DOUBLE_EQ(cmd.linear_velocity, 0.0);
}

// ============================================================================
// Control Output Tests
// ============================================================================

TEST_F(PathTrackerControllerTest, NoErrorZeroControl) {
    TrackingError error;
    error.cross_track_error = 0.0;
    error.heading_error = 0.0;
    error.velocity_error = 0.0;
    
    ControlCommand cmd = controller_->update(robot_state_, error);
    
    EXPECT_DOUBLE_EQ(cmd.angular_velocity, 0.0);
    EXPECT_DOUBLE_EQ(cmd.linear_velocity, 0.0);
}

TEST_F(PathTrackerControllerTest, CrossTrackErrorControl) {
    TrackingError error;
    error.cross_track_error = 0.1;  // 10cm lateral error
    error.heading_error = 0.0;
    error.velocity_error = 0.0;
    
    ControlCommand cmd = controller_->update(robot_state_, error);
    
    // Should produce non-zero steering
    EXPECT_NE(cmd.angular_velocity, 0.0);
}

TEST_F(PathTrackerControllerTest, VelocityErrorControl) {
    TrackingError error;
    error.cross_track_error = 0.0;
    error.heading_error = 0.0;
    error.velocity_error = 0.2;  // Desired speed is 0.2 m/s faster
    
    ControlCommand cmd = controller_->update(robot_state_, error);
    
    // Should produce non-zero linear velocity
    EXPECT_NE(cmd.linear_velocity, 0.0);
}

TEST_F(PathTrackerControllerTest, HeadingErrorControl) {
    TrackingError error;
    error.cross_track_error = 0.0;
    error.heading_error = 0.1;  // 0.1 rad heading error
    error.velocity_error = 0.0;
    
    ControlCommand cmd = controller_->update(robot_state_, error);
    
    // Should produce steering to align heading
    EXPECT_NE(cmd.angular_velocity, 0.0);
}

// ============================================================================
// PID Gains Tests
// ============================================================================

TEST_F(PathTrackerControllerTest, SetSteeringPIDGains) {
    EXPECT_NO_THROW(controller_->set_steering_pid_gains(1.5, 0.2, 0.3));
}

TEST_F(PathTrackerControllerTest, SetVelocityPIDGains) {
    EXPECT_NO_THROW(controller_->set_velocity_pid_gains(0.8, 0.1, 0.05));
}

TEST_F(PathTrackerControllerTest, DifferentPIDGainsBehavior) {
    TrackingError error;
    error.cross_track_error = 0.1;
    error.heading_error = 0.0;
    error.velocity_error = 0.0;
    
    // Set conservative gains
    controller_->set_steering_pid_gains(0.5, 0.05, 0.1);
    ControlCommand cmd1 = controller_->update(robot_state_, error);
    
    controller_->reset();
    
    // Set aggressive gains
    controller_->set_steering_pid_gains(2.0, 0.2, 0.4);
    ControlCommand cmd2 = controller_->update(robot_state_, error);
    
    // More aggressive should produce larger steering
    EXPECT_GT(std::abs(cmd2.angular_velocity), std::abs(cmd1.angular_velocity));
}

// ============================================================================
// Lookahead Distance Tests
// ============================================================================

TEST_F(PathTrackerControllerTest, LookaheadDistance) {
    double distance = controller_->compute_lookahead_distance(0.5);
    EXPECT_GT(distance, 0.0);
}

TEST_F(PathTrackerControllerTest, LookaheadIncreaseWithVelocity) {
    double distance_slow = controller_->compute_lookahead_distance(0.5);
    double distance_fast = controller_->compute_lookahead_distance(1.5);
    
    EXPECT_LT(distance_slow, distance_fast);
}

// ============================================================================
// Integral Windup Protection Tests
// ============================================================================

TEST_F(PathTrackerControllerTest, IntegralWindupPrevention) {
    TrackingError error;
    error.cross_track_error = 1.0;  // Large persistent error
    error.heading_error = 0.0;
    error.velocity_error = 0.0;
    
    // Apply same error multiple times
    for (int i = 0; i < 100; ++i) {
        ControlCommand cmd = controller_->update(robot_state_, error);
        
        // Angular velocity should be bounded
        EXPECT_LE(std::abs(cmd.angular_velocity), 0.5);
    }
}

// ============================================================================
// State Tracking Tests
// ============================================================================

TEST_F(PathTrackerControllerTest, MultipleUpdatesConsistency) {
    TrackingError error;
    error.cross_track_error = 0.05;
    error.heading_error = 0.02;
    error.velocity_error = 0.1;
    
    ControlCommand cmd1 = controller_->update(robot_state_, error);
    ControlCommand cmd2 = controller_->update(robot_state_, error);
    
    // Commands should differ due to integral accumulation
    EXPECT_NE(cmd1.angular_velocity, cmd2.angular_velocity);
}

// ============================================================================
// Saturation Tests
// ============================================================================

TEST_F(PathTrackerControllerTest, OutputSaturation) {
    TrackingError error;
    error.cross_track_error = 10.0;  // Huge lateral error
    error.heading_error = 1.0;
    error.velocity_error = 10.0;
    
    controller_->set_steering_pid_gains(10.0, 1.0, 1.0);
    controller_->set_velocity_pid_gains(10.0, 1.0, 1.0);
    
    ControlCommand cmd = controller_->update(robot_state_, error);
    
    // Should be saturated at max values
    EXPECT_LE(std::abs(cmd.angular_velocity), 0.5);
    EXPECT_LE(cmd.linear_velocity, 1.5);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
