#include "waypoint_follower/waypoint_follower_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<waypoint_follower::WaypointFollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
