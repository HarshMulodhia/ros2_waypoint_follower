#include <gtest/gtest.h>
#include "waypoint_follower/odometry_fusion.hpp"

using namespace waypoint_follower;

class OdometryFusionTest : public ::testing::Test {
protected:
    void SetUp() override {
        fusion = std::make_unique<OdometryFusion>(0.02);
        
        RobotState initial_state;
        initial_state.x = 0.0;
        initial_state.y = 0.0;
        initial_state.theta = 0.0;
        initial_state.vx = 0.0;
        initial_state.vy = 0.0;
        initial_state.omega = 0.0;
        
        fusion->initialize(initial_state);
    }

    std::unique_ptr<OdometryFusion> fusion;
};

TEST_F(OdometryFusionTest, InitializationTest) {
    RobotState state = fusion->get_fused_state();
    EXPECT_DOUBLE_EQ(state.x, 0.0);
    EXPECT_DOUBLE_EQ(state.y, 0.0);
    EXPECT_DOUBLE_EQ(state.theta, 0.0);
}

TEST_F(OdometryFusionTest, PredictFromOdometry) {
    fusion->predict_from_odometry(0.5, 0.0);
    RobotState state = fusion->get_fused_state();
    EXPECT_GT(state.x, 0.0);
    EXPECT_DOUBLE_EQ(state.y, 0.0);
}

TEST_F(OdometryFusionTest, UpdateFromIMU) {
    EXPECT_NO_THROW(fusion->update_from_imu(0.1, 0.0, 0.01));
}

TEST_F(OdometryFusionTest, UpdateFromEncoder) {
    EXPECT_NO_THROW(fusion->update_from_encoder(10.0, 10.0, 0.5, 0.1));
}

TEST_F(OdometryFusionTest, SetProcessNoise) {
    EXPECT_NO_THROW(fusion->set_process_noise(0.05));
}

TEST_F(OdometryFusionTest, SetMeasurementNoise) {
    EXPECT_NO_THROW(fusion->set_measurement_noise(0.2));
}

TEST_F(OdometryFusionTest, ResetFilter) {
    RobotState new_state;
    new_state.x = 5.0;
    new_state.y = 5.0;
    new_state.theta = 0.0;
    
    EXPECT_NO_THROW(fusion->reset(new_state));
    
    RobotState state = fusion->get_fused_state();
    EXPECT_DOUBLE_EQ(state.x, 5.0);
    EXPECT_DOUBLE_EQ(state.y, 5.0);
}

TEST_F(OdometryFusionTest, ContinuousPrediction) {
    for (int i = 0; i < 100; ++i) {
        fusion->predict_from_odometry(0.5, 0.1);
    }
    
    RobotState state = fusion->get_fused_state();
    EXPECT_GT(state.x, 0.0);
    EXPECT_GT(state.y, 0.0);
    EXPECT_LT(std::abs(state.theta), 2 * M_PI);
}
