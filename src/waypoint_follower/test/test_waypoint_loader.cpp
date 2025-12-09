#include <gtest/gtest.h>
#include "waypoint_follower/waypoint_loader.hpp"
#include <fstream>
#include <sstream>

using namespace waypoint_follower;

class WaypointLoaderTest : public ::testing::Test {
protected:
    void SetUp() override {
        loader = std::make_unique<WaypointLoader>();
        
        std::ofstream yaml_file("test_waypoints.yaml");
        yaml_file << "waypoints:\n";
        yaml_file << "  - x: 0.0\n    y: 0.0\n    velocity: 0.5\n";
        yaml_file << "  - x: 5.0\n    y: 5.0\n    velocity: 0.5\n";
        yaml_file << "  - x: 10.0\n    y: 0.0\n    velocity: 0.5\n";
        yaml_file.close();

        std::ofstream csv_file("test_waypoints.csv");
        csv_file << "x,y,velocity,theta\n";
        csv_file << "0.0,0.0,0.5,0.0\n";
        csv_file << "5.0,5.0,0.5,0.785\n";
        csv_file << "10.0,0.0,0.5,0.0\n";
        csv_file.close();
    }

    void TearDown() override {
        std::remove("test_waypoints.yaml");
        std::remove("test_waypoints.csv");
    }

    std::unique_ptr<WaypointLoader> loader;
};

TEST_F(WaypointLoaderTest, LoadFromYAML) {
    EXPECT_TRUE(loader->load_from_yaml("test_waypoints.yaml"));
    EXPECT_EQ(loader->size(), 3);
    
    const auto& wps = loader->get_waypoints();
    EXPECT_DOUBLE_EQ(wps[0].x, 0.0);
    EXPECT_DOUBLE_EQ(wps[0].y, 0.0);
    EXPECT_DOUBLE_EQ(wps[1].x, 5.0);
    EXPECT_DOUBLE_EQ(wps[1].y, 5.0);
}

TEST_F(WaypointLoaderTest, LoadFromCSV) {
    EXPECT_TRUE(loader->load_from_csv("test_waypoints.csv"));
    EXPECT_EQ(loader->size(), 3);
    
    const auto& wps = loader->get_waypoints();
    EXPECT_DOUBLE_EQ(wps[0].x, 0.0);
    EXPECT_DOUBLE_EQ(wps[2].x, 10.0);
}

TEST_F(WaypointLoaderTest, AddWaypoint) {
    loader->add_waypoint(Waypoint(1.0, 2.0, 0.5));
    EXPECT_EQ(loader->size(), 1);
}

TEST_F(WaypointLoaderTest, ValidateWaypoints) {
    loader->add_waypoint(Waypoint(0.0, 0.0, -0.5));
    EXPECT_TRUE(loader->validate());
    EXPECT_GE(loader->get_waypoints()[0].velocity, 0.0);
}

TEST_F(WaypointLoaderTest, EmptyWaypoints) {
    EXPECT_FALSE(loader->validate());
}
