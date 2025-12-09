#include <gtest/gtest.h>
#include "waypoint_follower/path_tracker_controller.hpp"

using namespace waypoint_follower;

class PathTrackerTest : public ::testing::Test {
protected:
    void SetUp() override {
        controller = std::make_unique<PathTrackerController>(0.02, 0.5, 1.5);
    }

    std::unique_ptr<PathTrackerController> controller;
};

TEST_F(PathTrackerTest, ControllerInitialization) {
    EXPECT_FALSE(controller->is_lqr_enabled());
}

TEST_F(PathTrackerTest, SetSteeringPIDGains) {
    EXPECT_NO_THROW(controller->set_steering_pid_gains(1.0, 0.1, 0.5));
}

TEST_F(PathTrackerTest, SetVelocityPIDGains) {
    EXPECT_NO_THROW(controller->set_velocity_pid_gains(0.5, 0.05, 0.1));
}

TEST_F(PathTrackerTest, EnableLQR) {
    controller->enable_lqr(true);
    EXPECT_TRUE(controller->is_lqr_enabled());
    
    controller->enable_lqr(false);
    EXPECT_FALSE(controller->is_lqr_enabled());
}

TEST_F(PathTrackerTest, ControllerUpdate) {
    RobotState state;
    state.x = 0.0;
    state.y = 0.0;
    state.theta = 0.0;
    state.vx = 0.5;
    state.vy = 0.0;
    state.omega = 0.0;

    TrackingError error;
    error.cross_track_error = 0.1;
    error.heading_error = 0.05;
    error.velocity_error = 0.1;
    error.progress = 0.5;

    ControlCommand cmd = controller->update(state, error);
    EXPECT_GE(cmd.linear_velocity, 0.0);
    EXPECT_LE(cmd.linear_velocity, 1.5);
    EXPECT_GE(cmd.angular_velocity, -0.5);
    EXPECT_LE(cmd.angular_velocity, 0.5);
}

TEST_F(PathTrackerTest, ResetController) {
    EXPECT_NO_THROW(controller->reset());
}

TEST_F(PathTrackerTest, ZeroError) {
    RobotState state;
    state.x = 0.0;
    state.y = 0.0;
    state.theta = 0.0;
    state.vx = 0.5;
    state.vy = 0.0;
    state.omega = 0.0;

    TrackingError error;
    error.cross_track_error = 0.0;
    error.heading_error = 0.0;
    error.velocity_error = 0.0;
    error.progress = 1.0;

    ControlCommand cmd = controller->update(state, error);
    EXPECT_GE(cmd.linear_velocity, 0.0);
    EXPECT_LE(std::abs(cmd.angular_velocity), 0.1);
}
