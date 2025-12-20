/**
 * @file test_waypoint_loader.cpp
 * @brief Comprehensive unit tests for WaypointLoader
 * @author Autonomous Vehicle Team
 * @date December 2025
 * @version 2.0.1
 */

#include <gtest/gtest.h>
#include <fstream>
#include <cstdio>
#include <memory>
#include "waypoint_follower/waypoint_loader.hpp"

using namespace waypoint_follower;

class WaypointLoaderTest : public ::testing::Test {
protected:
    void SetUp() override {
        loader_ = std::make_unique<WaypointLoader>();
        
        // Create test YAML file
        yaml_file_ = "test_waypoints.yaml";
        std::ofstream yaml(yaml_file_);
        yaml << "waypoints:\n";
        yaml << "  - x: 0.0\n    y: 0.0\n    velocity: 0.5\n    theta: 0.0\n";
        yaml << "  - x: 5.0\n    y: 5.0\n    velocity: 0.5\n    theta: 0.785\n";
        yaml << "  - x: 10.0\n    y: 0.0\n    velocity: 0.5\n    theta: 1.57\n";
        yaml.close();
        
        // Create test CSV file
        csv_file_ = "test_waypoints.csv";
        std::ofstream csv(csv_file_);
        csv << "x,y,velocity,theta\n";
        csv << "0.0,0.0,0.5,0.0\n";
        csv << "5.0,5.0,0.5,0.785\n";
        csv << "10.0,0.0,0.5,1.57\n";
        csv.close();
    }
    
    void TearDown() override {
        std::remove(yaml_file_.c_str());
        std::remove(csv_file_.c_str());
    }
    
    std::unique_ptr<WaypointLoader> loader_;
    std::string yaml_file_;
    std::string csv_file_;
};

// ============================================================================
// YAML Loading Tests
// ============================================================================

TEST_F(WaypointLoaderTest, LoadFromYAMLSuccess) {
    EXPECT_TRUE(loader_->load_from_yaml(yaml_file_));
    EXPECT_EQ(loader_->size(), 3);
}

TEST_F(WaypointLoaderTest, LoadFromYAMLVerifyWaypoints) {
    loader_->load_from_yaml(yaml_file_);
    const auto& wps = loader_->get_waypoints();
    
    EXPECT_DOUBLE_EQ(wps[0].x, 0.0);
    EXPECT_DOUBLE_EQ(wps[0].y, 0.0);
    EXPECT_DOUBLE_EQ(wps[0].velocity, 0.5);
    
    EXPECT_DOUBLE_EQ(wps[1].x, 5.0);
    EXPECT_DOUBLE_EQ(wps[1].y, 5.0);
    EXPECT_DOUBLE_EQ(wps[1].theta, 0.785);
    
    EXPECT_DOUBLE_EQ(wps[2].x, 10.0);
}

TEST_F(WaypointLoaderTest, LoadFromYAMLInvalidFile) {
    EXPECT_FALSE(loader_->load_from_yaml("nonexistent_file.yaml"));
}

// ============================================================================
// CSV Loading Tests
// ============================================================================

TEST_F(WaypointLoaderTest, LoadFromCSVSuccess) {
    EXPECT_TRUE(loader_->load_from_csv(csv_file_));
    EXPECT_EQ(loader_->size(), 3);
}

TEST_F(WaypointLoaderTest, LoadFromCSVVerifyWaypoints) {
    loader_->load_from_csv(csv_file_);
    const auto& wps = loader_->get_waypoints();
    
    EXPECT_DOUBLE_EQ(wps[0].x, 0.0);
    EXPECT_DOUBLE_EQ(wps[0].y, 0.0);
    EXPECT_DOUBLE_EQ(wps[2].x, 10.0);
}

TEST_F(WaypointLoaderTest, LoadFromCSVInvalidFile) {
    EXPECT_FALSE(loader_->load_from_csv("nonexistent_file.csv"));
}

// ============================================================================
// Waypoint Addition Tests
// ============================================================================

TEST_F(WaypointLoaderTest, AddSingleWaypoint) {
    Waypoint wp(1.0, 2.0, 0.5);
    loader_->add_waypoint(wp);
    EXPECT_EQ(loader_->size(), 1);
}

TEST_F(WaypointLoaderTest, AddMultipleWaypoints) {
    loader_->add_waypoint(Waypoint(0.0, 0.0, 0.5));
    loader_->add_waypoint(Waypoint(5.0, 5.0, 0.5));
    loader_->add_waypoint(Waypoint(10.0, 0.0, 0.5));
    EXPECT_EQ(loader_->size(), 3);
}

// ============================================================================
// Validation Tests
// ============================================================================

TEST_F(WaypointLoaderTest, ValidateEmptyWaypoints) {
    EXPECT_FALSE(loader_->validate());
}

TEST_F(WaypointLoaderTest, ValidateValidWaypoints) {
    loader_->load_from_yaml(yaml_file_);
    EXPECT_TRUE(loader_->validate());
}

TEST_F(WaypointLoaderTest, ValidateNegativeVelocity) {
    loader_->add_waypoint(Waypoint(0.0, 0.0, -0.5));
    EXPECT_TRUE(loader_->validate());
    EXPECT_GE(loader_->get_waypoints()[0].velocity, 0.0);
}

TEST_F(WaypointLoaderTest, ValidateInvalidCoordinates) {
    Waypoint wp(0.0, 0.0, 0.5);
    wp.x = std::numeric_limits<double>::infinity();
    loader_->add_waypoint(wp);
    EXPECT_FALSE(loader_->validate());
}

// ============================================================================
// Clear Tests
// ============================================================================

TEST_F(WaypointLoaderTest, ClearWaypoints) {
    loader_->load_from_yaml(yaml_file_);
    EXPECT_EQ(loader_->size(), 3);
    loader_->clear();
    EXPECT_EQ(loader_->size(), 0);
    EXPECT_FALSE(loader_->validate());
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST_F(WaypointLoaderTest, ReloadAfterClear) {
    loader_->load_from_yaml(yaml_file_);
    EXPECT_EQ(loader_->size(), 3);
    
    loader_->clear();
    EXPECT_EQ(loader_->size(), 0);
    
    loader_->load_from_csv(csv_file_);
    EXPECT_EQ(loader_->size(), 3);
}

TEST_F(WaypointLoaderTest, MixedLoading) {
    loader_->add_waypoint(Waypoint(0.0, 0.0, 0.5));
    loader_->load_from_yaml(yaml_file_);
    // Should replace with YAML content
    EXPECT_EQ(loader_->size(), 3);
}

// ============================================================================
// Performance Tests
// ============================================================================

TEST_F(WaypointLoaderTest, LargeWaypointSet) {
    for (int i = 0; i < 1000; ++i) {
        loader_->add_waypoint(Waypoint(i * 1.0, i * 1.0, 0.5));
    }
    EXPECT_EQ(loader_->size(), 1000);
    EXPECT_TRUE(loader_->validate());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
