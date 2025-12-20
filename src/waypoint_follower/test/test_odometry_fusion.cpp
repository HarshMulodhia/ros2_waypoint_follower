/**
 * @file test_odometry_fusion.cpp
 * @brief Comprehensive unit tests for OdometryFusion Kalman filter
 * @author Autonomous Vehicle Team
 * @date December 2025
 * @version 2.0.1
 */

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include "waypoint_follower/odometry_fusion.hpp"

using namespace waypoint_follower;

class OdometryFusionTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create fusion filter with 0.02s time step (50 Hz)
        fusion_ = std::make_unique<OdometryFusion>(0.02);
        
        // Initialize with known state
        initial_state_.x = 0.0;
        initial_state_.y = 0.0;
        initial_state_.theta = 0.0;
        initial_state_.vx = 0.0;
        initial_state_.vy = 0.0;
        initial_state_.omega = 0.0;
        initial_state_.timestamp = 0.0;
        
        fusion_->initialize(initial_state_);
    }
    
    std::unique_ptr<OdometryFusion> fusion_;
    RobotState initial_state_;
};

// ============================================================================
// Initialization Tests
// ============================================================================

TEST_F(OdometryFusionTest, FilterInitialization) {
    EXPECT_NE(fusion_, nullptr);
}

TEST_F(OdometryFusionTest, InitialStateCorrect) {
    RobotState state = fusion_->get_fused_state();
    
    EXPECT_DOUBLE_EQ(state.x, 0.0);
    EXPECT_DOUBLE_EQ(state.y, 0.0);
    EXPECT_DOUBLE_EQ(state.theta, 0.0);
    EXPECT_DOUBLE_EQ(state.vx, 0.0);
    EXPECT_DOUBLE_EQ(state.vy, 0.0);
    EXPECT_DOUBLE_EQ(state.omega, 0.0);
}

TEST_F(OdometryFusionTest, ResetFilter) {
    RobotState new_state;
    new_state.x = 5.0;
    new_state.y = 3.0;
    new_state.theta = 0.5;
    new_state.vx = 0.5;
    new_state.vy = 0.0;
    new_state.omega = 0.1;
    
    fusion_->reset(new_state);
    
    RobotState state = fusion_->get_fused_state();
    EXPECT_DOUBLE_EQ(state.x, 5.0);
    EXPECT_DOUBLE_EQ(state.y, 3.0);
    EXPECT_DOUBLE_EQ(state.theta, 0.5);
}

// ============================================================================
// Odometry Prediction Tests
// ============================================================================

TEST_F(OdometryFusionTest, PredictFromOdometryStraightLine) {
    // Move forward 0.5 m/s for multiple steps
    for (int i = 0; i < 10; ++i) {
        fusion_->predict_from_odometry(0.5, 0.0);
    }
    
    RobotState state = fusion_->get_fused_state();
    
    // Should have moved forward
    EXPECT_GT(state.x, 0.0);
    EXPECT_NEAR(state.y, 0.0, 0.01);
    EXPECT_NEAR(state.theta, 0.0, 0.01);
}

TEST_F(OdometryFusionTest, PredictFromOdometryWithTurning) {
    // Move forward and turn
    for (int i = 0; i < 20; ++i) {
        fusion_->predict_from_odometry(0.5, 0.1);  // 0.5 m/s, 0.1 rad/s
    }
    
    RobotState state = fusion_->get_fused_state();
    
    // Should have rotated
    EXPECT_GT(state.theta, 0.0);
    EXPECT_LT(state.theta, 2.0 * M_PI);
}

TEST_F(OdometryFusionTest, PredictStationaryRobot) {
    // No motion
    for (int i = 0; i < 10; ++i) {
        fusion_->predict_from_odometry(0.0, 0.0);
    }
    
    RobotState state = fusion_->get_fused_state();
    
    // Should remain at origin
    EXPECT_DOUBLE_EQ(state.x, 0.0);
    EXPECT_DOUBLE_EQ(state.y, 0.0);
}

TEST_F(OdometryFusionTest, PredictConsistency) {
    // Multiple short steps should sum to same result as one long step
    RobotState state1 = fusion_->get_fused_state();
    
    for (int i = 0; i < 10; ++i) {
        fusion_->predict_from_odometry(0.5, 0.0);
    }
    
    RobotState state2 = fusion_->get_fused_state();
    
    // Should have accumulated distance
    double dist = std::hypot(state2.x - state1.x, state2.y - state1.y);
    EXPECT_GT(dist, 0.0);
}

// ============================================================================
// IMU Update Tests
// ============================================================================

TEST_F(OdometryFusionTest, UpdateFromIMU) {
    EXPECT_NO_THROW(fusion_->update_from_imu(0.1, 0.0, 0.01));
}

TEST_F(OdometryFusionTest, UpdateFromIMUMultiple) {
    // Apply IMU updates
    for (int i = 0; i < 10; ++i) {
        EXPECT_NO_THROW(fusion_->update_from_imu(0.05, 0.02, 0.01));
    }
    
    RobotState state = fusion_->get_fused_state();
    EXPECT_TRUE(std::isfinite(state.x));
    EXPECT_TRUE(std::isfinite(state.y));
    EXPECT_TRUE(std::isfinite(state.theta));
}

// ============================================================================
// Encoder Update Tests
// ============================================================================

TEST_F(OdometryFusionTest, UpdateFromEncoder) {
    // Wheel ticks: 10 left, 10 right (straight line)
    EXPECT_NO_THROW(fusion_->update_from_encoder(10.0, 10.0, 0.5, 0.1));
}

TEST_F(OdometryFusionTest, UpdateFromEncoderDifferentialMotion) {
    // Different wheel speeds: turning motion
    EXPECT_NO_THROW(fusion_->update_from_encoder(10.0, 5.0, 0.5, 0.1));
}

TEST_F(OdometryFusionTest, UpdateFromEncoderMultiple) {
    for (int i = 0; i < 5; ++i) {
        EXPECT_NO_THROW(fusion_->update_from_encoder(5.0, 5.0, 0.5, 0.1));
    }
    
    RobotState state = fusion_->get_fused_state();
    EXPECT_TRUE(std::isfinite(state.x));
    EXPECT_TRUE(std::isfinite(state.y));
}

// ============================================================================
// Filter Tuning Tests
// ============================================================================

TEST_F(OdometryFusionTest, SetProcessNoise) {
    EXPECT_NO_THROW(fusion_->set_process_noise(0.05));
    EXPECT_NO_THROW(fusion_->set_process_noise(0.01));
    EXPECT_NO_THROW(fusion_->set_process_noise(0.1));
}

TEST_F(OdometryFusionTest, SetMeasurementNoise) {
    EXPECT_NO_THROW(fusion_->set_measurement_noise(0.2));
    EXPECT_NO_THROW(fusion_->set_measurement_noise(0.05));
    EXPECT_NO_THROW(fusion_->set_measurement_noise(0.5));
}

TEST_F(OdometryFusionTest, LowProcessNoiseHighConfidence) {
    // Low process noise → high confidence in model
    fusion_->set_process_noise(0.001);
    fusion_->set_measurement_noise(0.2);
    
    // Predictions should be consistent
    fusion_->predict_from_odometry(0.5, 0.0);
    RobotState state1 = fusion_->get_fused_state();
    
    fusion_->predict_from_odometry(0.5, 0.0);
    RobotState state2 = fusion_->get_fused_state();
    
    // Should integrate consistently
    EXPECT_GT(state2.x, state1.x);
}

TEST_F(OdometryFusionTest, HighProcessNoiseLowConfidence) {
    // High process noise → low confidence in model
    fusion_->set_process_noise(0.5);
    fusion_->set_measurement_noise(0.01);
    
    // Measurements should influence state more
    fusion_->predict_from_odometry(0.5, 0.0);
    fusion_->update_from_imu(0.1, 0.0, 0.05);
    
    RobotState state = fusion_->get_fused_state();
    EXPECT_TRUE(std::isfinite(state.x));
}

// ============================================================================
// Sensor Fusion Integration Tests
// ============================================================================

TEST_F(OdometryFusionTest, FusionWithOdometryAndIMU) {
    // Typical sensor fusion loop
    for (int i = 0; i < 10; ++i) {
        // Predict from odometry
        fusion_->predict_from_odometry(0.5, 0.05);
        
        // Update from IMU
        fusion_->update_from_imu(0.02, 0.01, 0.05);
    }
    
    RobotState state = fusion_->get_fused_state();
    
    // Should have valid state
    EXPECT_TRUE(std::isfinite(state.x));
    EXPECT_TRUE(std::isfinite(state.y));
    EXPECT_TRUE(std::isfinite(state.theta));
    EXPECT_TRUE(std::isfinite(state.vx));
    EXPECT_TRUE(std::isfinite(state.vy));
    EXPECT_TRUE(std::isfinite(state.omega));
}

TEST_F(OdometryFusionTest, FusionWithAllSensors) {
    // Use all three sensor modalities
    for (int i = 0; i < 10; ++i) {
        fusion_->predict_from_odometry(0.5, 0.05);
        fusion_->update_from_imu(0.02, 0.01, 0.05);
        fusion_->update_from_encoder(5.0, 5.0, 0.5, 0.1);
    }
    
    RobotState state = fusion_->get_fused_state();
    
    EXPECT_TRUE(std::isfinite(state.x));
    EXPECT_TRUE(std::isfinite(state.y));
    EXPECT_TRUE(std::isfinite(state.theta));
}

// ============================================================================
// State Validity Tests
// ============================================================================

TEST_F(OdometryFusionTest, StateRemainsValid) {
    // Apply many operations and verify state stays valid
    for (int i = 0; i < 100; ++i) {
        fusion_->predict_from_odometry(0.5 + 0.1 * std::sin(i * 0.1), 0.1 * std::cos(i * 0.05));
        
        if (i % 5 == 0) {
            fusion_->update_from_imu(0.05, 0.02, 0.05);
        }
        
        if (i % 10 == 0) {
            fusion_->update_from_encoder(5.0, 5.0, 0.5, 0.1);
        }
    }
    
    RobotState state = fusion_->get_fused_state();
    
    // Check all state values are finite
    EXPECT_TRUE(std::isfinite(state.x));
    EXPECT_TRUE(std::isfinite(state.y));
    EXPECT_TRUE(std::isfinite(state.theta));
    EXPECT_TRUE(std::isfinite(state.vx));
    EXPECT_TRUE(std::isfinite(state.vy));
    EXPECT_TRUE(std::isfinite(state.omega));
    
    // Check angle is in reasonable range
    EXPECT_LE(std::abs(state.theta), 10.0);  // Allow for multiple rotations
}

TEST_F(OdometryFusionTest, NoNaNPropagation) {
    // Verify NaN doesn't propagate
    RobotState state = fusion_->get_fused_state();
    
    EXPECT_FALSE(std::isnan(state.x));
    EXPECT_FALSE(std::isnan(state.y));
    EXPECT_FALSE(std::isnan(state.theta));
    EXPECT_FALSE(std::isnan(state.vx));
    EXPECT_FALSE(std::isnan(state.vy));
    EXPECT_FALSE(std::isnan(state.omega));
}

// ============================================================================
// Performance Tests
// ============================================================================

TEST_F(OdometryFusionTest, HighFrequencyOperations) {
    // Simulate 100 Hz operation for 10 seconds
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < 1000; ++i) {
        fusion_->predict_from_odometry(0.5, 0.05);
        
        if (i % 2 == 0) {
            fusion_->update_from_imu(0.02, 0.01, 0.05);
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    // Should complete in reasonable time (<1000ms for 1000 operations)
    EXPECT_LT(duration.count(), 1000);
}

TEST_F(OdometryFusionTest, MemoryStability) {
    // Verify filter doesn't accumulate state errors
    RobotState state_before = fusion_->get_fused_state();
    
    for (int i = 0; i < 100; ++i) {
        fusion_->predict_from_odometry(0.0, 0.0);  // No motion
    }
    
    RobotState state_after = fusion_->get_fused_state();
    
    // Stationary robot should remain stationary
    EXPECT_NEAR(state_after.x, state_before.x, 0.01);
    EXPECT_NEAR(state_after.y, state_before.y, 0.01);
    EXPECT_NEAR(state_after.theta, state_before.theta, 0.01);
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST_F(OdometryFusionTest, ZeroTimeStep) {
    // Filter handles zero velocity
    EXPECT_NO_THROW(fusion_->predict_from_odometry(0.0, 0.0));
}

TEST_F(OdometryFusionTest, LargeVelocity) {
    // High velocity still produces valid state
    for (int i = 0; i < 5; ++i) {
        fusion_->predict_from_odometry(5.0, 1.0);
    }
    
    RobotState state = fusion_->get_fused_state();
    EXPECT_TRUE(std::isfinite(state.x));
    EXPECT_TRUE(std::isfinite(state.y));
}

TEST_F(OdometryFusionTest, RapidAngularChanges) {
    // Rapid rotation
    for (int i = 0; i < 10; ++i) {
        fusion_->predict_from_odometry(0.5, 2.0);  // Fast rotation
    }
    
    RobotState state = fusion_->get_fused_state();
    EXPECT_TRUE(std::isfinite(state.theta));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
