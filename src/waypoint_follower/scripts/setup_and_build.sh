#!/bin/bash

################################################################################
# Autonomous Waypoint Follower - Complete Setup & Build Guide
#
# File: setup_and_build.sh
# Purpose: Automated setup and build for waypoint_follower package
# Requirements: ROS2 Jazzy installed
# Usage: bash setup_and_build.sh
#
# What this script does:
# 1. Creates directory structure
# 2. Sets up package dependencies
# 3. Copies all files to correct locations
# 4. Builds the package
# 5. Runs unit tests
# 6. Provides launch instructions
#
# Last Updated: December 21, 2025
################################################################################

set -e  # Exit on any error

# Color codes for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Autonomous Waypoint Follower Setup${NC}"
echo -e "${BLUE}========================================${NC}\n"

# ============================================================================
# SETUP WORKSPACE
# ============================================================================

echo -e "${YELLOW}[1/6]${NC} Setting up workspace..."

# Create directory structure
mkdir -p ~/ros2_ws/src/waypoint_follower/{include/waypoint_follower,src,config,launch,scripts,tests}

WS_DIR="$HOME/ros2_ws"
PKG_DIR="$WS_DIR/src/waypoint_follower"

echo -e "${GREEN}✓${NC} Workspace created at $WS_DIR"

# ============================================================================
# COPY HEADER FILES
# ============================================================================

echo -e "${YELLOW}[2/6]${NC} Installing header files..."

cat > "$PKG_DIR/include/waypoint_follower/path_tracker_controller.hpp" << 'HEADER_1'
[Insert path_tracker_controller.hpp content here]
HEADER_1

cat > "$PKG_DIR/include/waypoint_follower/waypoint_loader.hpp" << 'HEADER_2'
[Insert waypoint_loader.hpp content here]
HEADER_2

cat > "$PKG_DIR/include/waypoint_follower/trajectory_smoother.hpp" << 'HEADER_3'
[Insert trajectory_smoother.hpp content here]
HEADER_3

cat > "$PKG_DIR/include/waypoint_follower/odometry_fusion.hpp" << 'HEADER_4'
[Insert odometry_fusion.hpp content here]
HEADER_4

echo -e "${GREEN}✓${NC} Header files installed"

# ============================================================================
# COPY CONFIGURATION FILES
# ============================================================================

echo -e "${YELLOW}[3/6]${NC} Installing configuration files..."

# Parameters YAML
cp waypoint_follower_params.yaml "$PKG_DIR/config/"

# Waypoints YAML
cp waypoints_course.yaml "$PKG_DIR/config/"

# CMakeLists
cp CMakeLists.txt "$PKG_DIR/"

# package.xml
cp package.xml "$PKG_DIR/"

echo -e "${GREEN}✓${NC} Configuration files installed"

# ============================================================================
# COPY PYTHON SCRIPTS
# ============================================================================

echo -e "${YELLOW}[4/6]${NC} Installing Python scripts..."

cp physics_simulator.py "$PKG_DIR/scripts/"
chmod +x "$PKG_DIR/scripts/physics_simulator.py"

echo -e "${GREEN}✓${NC} Python scripts installed"

# ============================================================================
# BUILD PACKAGE
# ============================================================================

echo -e "${YELLOW}[5/6]${NC} Building package..."

cd "$WS_DIR"
source /opt/ros/humble/setup.bash

echo "Building with: colcon build --packages-select waypoint_follower"
colcon build --packages-select waypoint_follower

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓${NC} Build successful"
else
    echo -e "${RED}✗${NC} Build failed"
    exit 1
fi

# ============================================================================
# RUN TESTS
# ============================================================================

echo -e "${YELLOW}[6/6]${NC} Running unit tests..."

colcon test --packages-select waypoint_follower

echo -e "${GREEN}✓${NC} Tests completed"

# ============================================================================
# SUCCESS MESSAGE
# ============================================================================

echo -e "\n${GREEN}========================================${NC}"
echo -e "${GREEN}Setup Complete!${NC}"
echo -e "${GREEN}========================================${NC}\n"

echo -e "${BLUE}Next Steps:${NC}"
echo ""
echo "1. Source the workspace:"
echo -e "   ${YELLOW}source $WS_DIR/install/setup.bash${NC}"
echo ""
echo "2. Verify waypoints are loaded:"
echo -e "   ${YELLOW}cat $PKG_DIR/config/waypoints_course.yaml${NC}"
echo ""
echo "3. Launch the system:"
echo -e "   ${YELLOW}ros2 launch waypoint_follower waypoint_follower.launch.py${NC}"
echo ""
echo "4. In another terminal, send a navigation goal:"
echo -e "   ${YELLOW}ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints \\${NC}"
echo -e "   ${YELLOW}'{ poses: [{pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, \\${NC}"
echo -e "   ${YELLOW}{pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}]}'${NC}"
echo ""

# ============================================================================
# CONFIGURATION SUMMARY
# ============================================================================

echo -e "${BLUE}System Configuration:${NC}"
echo ""
echo "Robot Specifications:"
echo "  - Size: 0.8m × 0.5m × 0.4m"
echo "  - Weight: 12kg"
echo "  - Wheelbase: 0.5m"
echo "  - Wheel Radius: 0.1m"
echo ""
echo "Control Parameters:"
echo "  - Max Linear Velocity: 0.3 m/s"
echo "  - Max Steering Angle: 0.5 rad"
echo "  - Max Curvature: 1.5 1/m"
echo "  - Min Turn Radius: 0.67m"
echo "  - Control Loop Rate: 50Hz"
echo ""
echo "Performance Targets:"
echo "  - Tracking Error: < 0.1m"
echo "  - Cross-track Error: < 0.05m"
echo "  - Heading Accuracy: < 0.1 rad"
echo "  - Control Latency: < 30ms"
echo ""

echo -e "${GREEN}Ready for testing!${NC}\n"
