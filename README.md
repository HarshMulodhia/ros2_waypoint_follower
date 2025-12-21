# ROS2 Waypoint Follower

**Professional-Grade Autonomous Navigation System for Wheeled Robots**

[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-blue.svg)](https://docs.ros.org/)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![Status](https://img.shields.io/badge/status-Production%20Ready-brightgreen.svg)]()

## Overview

ROS2 Waypoint Follower is a complete autonomous navigation framework for differential-drive and holonomic robots. It provides:

✅ **Path Planning** - Smooth trajectory generation from discrete waypoints  
✅ **Path Tracking** - Dual-loop PID controller for precise path following  
✅ **State Estimation** - Extended Kalman Filter for multi-sensor odometry fusion  
✅ **Physics Simulation** - Built-in differential drive simulator for testing  
✅ **ROS 2 Integration** - Native action server for goal-based navigation  

---

## Features

### Core Capabilities

| Feature | Description |
|---------|-------------|
| **Trajectory Smoothing** | Linear interpolation between waypoints with configurable density |
| **Curvature-Aware Planning** | Computes path curvature and limits to robot's turning radius |
| **Velocity Profiling** | Trapezoidal acceleration profile (smooth ramp-up/down) |
| **Cross-Track Control** | PID-based lateral error correction with look-ahead distance |
| **Heading Alignment** | Proportional heading error control for orientation accuracy |
| **Multi-Sensor Fusion** | Kalman filter fuses odometry, IMU, and wheel encoders |
| **Real-Time Control** | 50 Hz control loop with soft-realtime guarantees |
| **ROS 2 Action Server** | Standard navigation action interface (nav2_msgs/FollowWaypoints) |

### Unique Strengths

- **Built from scratch** without Nav2/Navfn dependencies for lightweight deployment
- **Custom robotics_control library** integration for advanced control algorithms
- **Comprehensive documentation** with algorithmic explanations and implementation details
- **Production-tested** on real differential-drive platforms
- **Extensible architecture** for adding LQR, MPC, or other controllers

---

## System Architecture

### Component Hierarchy

```
WaypointFollowerNode (main ROS 2 node)
├── WaypointLoader (YAML/CSV file parser)
├── TrajectorySmoother (waypoint interpolation & velocity profile)
├── PathTrackerController (dual PID controller)
├── OdometryFusion (Kalman filter state estimator)
└── PhysicsSimulator (physics-based robot simulator)
```

### Data Flow

```
User Goal (action)
    ↓
WaypointLoader → Parse poses
    ↓
TrajectorySmoother → Generate trajectory
    ↓
Main Control Loop (50 Hz)
├── Receive /odom → Update robot_state
├── Compute tracking error
├── PathTrackerController → Generate /cmd_vel
└── Publish /odometry/fused, /path_visualization
    ↓
PhysicsSimulator (or real hardware)
    ↓
Feedback: /odom → Odometry topic
```

### Finite State Machine

```
IDLE --[goal received]--> PLANNING --[trajectory generated]--> TRACKING
 ↑                                                                  │
 │                                                                  ├─[goal reached]─→ GOAL_REACHED
 │                                                                  │                       │
 └────────────────────────────[success/cancel]─────────────────────┴─ IDLE
                                                                        ↑
                                                          ERROR ────────┘
```

---

## Quick Start

### 1. Prerequisites

```bash
# Ubuntu 24.04 with ROS 2 Jazzy
sudo apt-get update
cd ~/ros2_ws/
rosdep install --from-paths src --ignore-src -y

# Build robotics_control dependency
cd ~/ros2_ws/
git clone https://github.com/HarshMulodhia/robotics_control_cpp.git
cd ~/ros2_ws && colcon build --packages-select robotics_control
```

### 2. Clone and Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/HarshMulodhia/ros2_waypoint_follower.git
cd ~/ros2_ws

# Build waypoint_follower
colcon build --packages-select waypoint_follower --symlink-install
source install/setup.bash
```

### 3. Run Demo

```bash
# Terminal 1: Launch system with simulator
ros2 launch waypoint_follower waypoint_follower.launch.py use_rviz:=true use_simulator:=true

# Terminal 2: Send navigation goal

ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints \
'{"poses":[
  {"pose":{"position":{"x":0.0,"y":0.0,"z":0.0},"orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0}}},
  {"pose":{"position":{"x":5.0,"y":0.0,"z":0.0},"orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0}}},
  {"pose":{"position":{"x":10.0,"y":0.0,"z":0.0},"orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0}}},
  {"pose":{"position":{"x":15.0,"y":0.0,"z":0.0},"orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0}}},
  {"pose":{"position":{"x":20.0,"y":0.0,"z":0.0},"orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0}}}
],"goal_index":0,"number_of_loops":0}'

# Terminal 3: Monitor robot
ros2 topic echo /odom
rviz2 # Visualize path and robot motion
```

---

## Configuration

### Main Parameters (`waypoint_follower_params.yaml`)

```yaml
# Steering (cross-track error) PID Controller
path_tracker:
  steering_pid:
    kp: 1.0      # Proportional: stronger → faster lateral correction
    ki: 0.1      # Integral: remove steady-state lateral bias
    kd: 0.5      # Derivative: damping for smoother response
  
  # Velocity (speed) PID Controller
  velocity_pid:
    kp: 0.5      # Proportional: track desired velocity
    ki: 0.05     # Integral: small to avoid windup
    kd: 0.1      # Derivative: smooth acceleration changes
  
  # Physical constraints
  max_steering_angle: 0.5      # Maximum angular velocity (rad/s)
  max_linear_velocity: 1.5     # Maximum forward velocity (m/s)

# Path smoothing
trajectory:
  max_curvature: 0.5           # Maximum allowed curvature (1/meters)
  interpolation_distance: 0.1  # Point spacing in trajectory (meters)

# Sensor fusion (Kalman filter)
odometry_fusion:
  process_noise_q: 0.01        # Trust in motion model (lower = more drift)
  measurement_noise_r: 0.1     # Trust in measurements (lower = more responsive)
```

### Waypoints File (`waypoints_course.yaml`)

```yaml
waypoints:
  # Waypoint 1: Move forward to (5,0)
  - x: 5.0
    y: 0.0
    velocity: 0.5              # Cruise speed (m/s)
    theta: 0.0                 # Desired heading (optional)
  
  # Waypoint 2: Turn right to (5,5)
  - x: 5.0
    y: 5.0
    velocity: 0.3              # Slower around turn
    theta: 1.5708              # π/2 radians (90°)
  
  # Waypoint 3: Return to origin
  - x: 0.0
    y: 0.0
    velocity: 0.5
```

### Launch Arguments

```bash
# Customize launch behavior
ros2 launch waypoint_follower waypoint_follower.launch.py \
  use_rviz:=true \               # Enable RViz visualization
  use_simulator:=true \          # Use physics simulator instead of real robot
  use_sim_time:=false \          # Use system time (true for rosbag playback)
  waypoints_file:=/path/to/waypoints.yaml
```

---

## API Reference

### ROS 2 Interfaces

#### Action Server: `/follow_waypoints`

**Type:** `nav2_msgs/action/FollowWaypoints`

**Request (Goal):**
```python
geometry_msgs.msg.PoseStamped[] poses  # Array of target waypoints
```

**Response (Result):**
```python
std_msgs.msg.Empty result              # Empty on success
```

**Cancellation:** Accepted at any time, robot stops immediately

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Robot odometry (from physics simulator or real odometry) |
| `/imu/data` | `sensor_msgs/Imu` | IMU measurements for Kalman filter fusion |

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands for robot motor controllers |
| `/odometry/fused` | `nav_msgs/Odometry` | Fused state estimate from Kalman filter |
| `/path_visualization` | `nav_msgs/Path` | Current trajectory for RViz visualization |

#### Transform Broadcasts

| Source | Target | Description |
|--------|--------|-------------|
| `map` | `odom` | Static transform (identity) |
| `odom` | `base_link` | Robot pose in odometry frame |

---

## Algorithm Details

### Path Tracking Control

**Cross-Track Error PID:**
```
ω = Kp * e_cte + Ki * ∫e_cte + Kd * de_cte/dt

where:
  ω = angular velocity command
  e_cte = cross-track error (perpendicular distance to path)
  Kp, Ki, Kd = tuned gains
```

**Velocity PID:**
```
v = Kp * e_vel + Ki * ∫e_vel + Kd * de_vel/dt

where:
  v = linear velocity command
  e_vel = velocity error (desired - current)
```

### Cross-Track Error Computation

```
1. Find nearest trajectory point to robot
2. Use next point for path heading: θ_path = atan2(Δy, Δx)
3. Perpendicular distance: e_cte = -dx*sin(θ_path) + dy*cos(θ_path)
   (negative = left of path, positive = right of path)
```

### Kalman Filter (6-State)

**State Vector:**
```
x = [x, y, θ, vx, vy, ω]^T
```

**Prediction:** Kinematic model
```
x[k+1] = x[k] + vx*cos(θ)*dt
y[k+1] = y[k] + vx*sin(θ)*dt
θ[k+1] = θ[k] + ω*dt
```

**Update:** Measurement fusion
```
Measurements: [ω_gyro, ax, ay]
Measurement matrix C maps state to expected measurements
```

### Trajectory Smoothing

**Algorithm:**
```
1. Convert waypoints to trajectory points
2. Linear interpolation between consecutive waypoints
   - Interpolation distance: 0.1m (configurable)
3. Compute arc length: s = cumulative distance along path
4. Calculate curvature: κ = dθ/ds (clamp to max_curvature)
5. Generate trapezoidal velocity profile:
   - Acceleration (0-25%): ramp 0 → max_velocity
   - Cruise (25-75%): constant max_velocity
   - Deceleration (75-100%): ramp max_velocity → 0
```

---

## Performance Tuning

### Parameter Tuning Guide

#### For Faster Path Tracking
```yaml
path_tracker:
  steering_pid:
    kp: 1.5      # Increase proportional gain
    ki: 0.15
    kd: 0.8
```

#### For Smoother Motion (Less Oscillation)
```yaml
path_tracker:
  steering_pid:
    kp: 0.7      # Decrease proportional gain
    ki: 0.05
    kd: 0.3
```

#### For Better Velocity Tracking
```yaml
path_tracker:
  velocity_pid:
    kp: 0.8      # Increase proportional gain
    ki: 0.1
    kd: 0.2
```

#### For Improved State Estimation (More Responsive)
```yaml
odometry_fusion:
  process_noise_q: 0.05    # Less trust in motion model
  measurement_noise_r: 0.05 # More trust in sensors
```

### Debugging Tools

```bash
# Monitor PID controller outputs
ros2 topic echo /cmd_vel

# Check tracking error in real-time
ros2 run waypoint_follower waypoint_follower_node --ros-args --log-level DEBUG

# Plot trajectory
ros2 topic echo /path_visualization

# Visualize in RViz
rviz2
```

---

## File Structure

```
src
    ├── robotics_control
    │   ├── CMakeLists.txt
    │   ├── LICENSE
    │   ├── include
    │   │   └── robotics_control
    │   │       ├── control
    │   │       │   ├── lqr_controller.hpp
    │   │       │   ├── pid_controller.hpp
    │   │       │   └── state_space.hpp
    │   │       ├── estimation
    │   │       │   └── kalman_filter.hpp
    │   │       └── maths
    │   │           └── transform.h
    │   ├── package.xml
    │   └── src
    │       ├── CMakeLists.txt
    │       ├── kalman_filter.cpp
    │       ├── lqr_controller.cpp
    │       ├── main.cpp
    │       ├── pid_controller.cpp
    │       ├── state_space.cpp
    │       └── transform.cpp
    └── waypoint_follower
        ├── CMakeLists.txt
        ├── config
        │   ├── goal_poses.txt
        │   ├── nav2_params.yaml
        │   ├── path_tracking.yaml
        │   ├── robot.urdf
        │   ├── warehouse.world
        │   ├── waypoint_follower_params.yaml
        │   ├── waypoints_course.yaml
        │   └── waypoints_default_path.hpp.in
        ├── include
        │   └── waypoint_follower
        │       ├── odometry_fusion.hpp
        │       ├── path_tracker_controller.hpp
        │       ├── trajectory_smoother.hpp
        │       ├── waypoint_follower_node.hpp
        │       ├── waypoint_loader.hpp
        │       └── waypoint_types.hpp
        ├── launch
        │   ├── gazebo_sim.launch.py
        │   ├── sim.launch.py
        │   └── waypoint_follower.launch.py
        ├── package.xml
        ├── rviz
        │   └── waypoint_follower.rviz
        ├── scripts
        │   └── physics_simulator.py
        ├── src
        │   ├── odometry_fusion.cpp
        │   ├── path_tracker_controller.cpp
        │   ├── trajectory_smoother.cpp
        │   ├── waypoint_follower_node.cpp
        │   ├── waypoint_follower_node_main.cpp
        │   └── waypoint_loader.cpp
        └── test
            ├── test_odometry_fusion.cpp
            ├── test_path_tracker.cpp
            ├── test_trajectory_smoother.cpp
            └── test_waypoint_loader.cpp
```

---

## Integration with Real Hardware

### Connecting to Real Robot

**Odometry Source:**
```bash
# If robot publishes to /odom with frame_id "odom":
# → No changes needed, remapping optional

# If robot publishes to different topic:
ros2 launch waypoint_follower waypoint_follower.launch.py \
  --ros-args -r /odom:=/robot/odom
```

**Motor Control:**
```bash
# If robot subscribes to /cmd_vel:
# → Waypoint follower works out-of-the-box

# If robot uses different topic:
# → Create a simple adapter node:
# 
# ros2 topic pub /robot/motor_command geometry_msgs/Twist \
#   "$(ros2 topic echo /cmd_vel -1)"
```

**Coordinate Frame Setup:**
```bash
# Ensure TF tree is published:
map → odom → base_link

# Use robot_state_publisher:
ros2 launch your_robot_bringup robot.launch.py

# The waypoint follower will use published odom transform
```

---

## Troubleshooting

### Robot Not Following Path

**Check 1: Is /cmd_vel being published?**
```bash
ros2 topic hz /cmd_vel
# Should show 50.0 Hz
```

**Check 2: Is /odom being received?**
```bash
ros2 topic hz /odom
# Should show similar rate to your odometry source
```

**Check 3: Tune PID gains**
```yaml
# Start with conservative gains
steering_pid:
  kp: 0.5
  ki: 0.05
  kd: 0.2
# Then increase gradually
```

### Robot Oscillates Around Path

**Solution:** Reduce derivative gain and increase damping
```yaml
steering_pid:
  kp: 1.0      # Keep moderate
  ki: 0.1
  kd: 0.3      # Increase damping
```

### Robot Overshoots Goal

**Solution:** Reduce proportional gain and increase deceleration
```yaml
steering_pid:
  kp: 0.8      # Lower gain
  ki: 0.08
  kd: 0.4
trajectory:
  interpolation_distance: 0.05  # Denser trajectory
```

### No TF Transform Between Frames

**Solution:** Ensure physics simulator or robot_state_publisher is running
```bash
ros2 tf2_tree  # Visualize TF tree
# Should show: map → odom → base_link
```

---

## Dependencies

### Runtime

| Package | Version | Purpose |
|---------|---------|---------|
| `robotics_control` | Latest | PID controller, Kalman filter |
| `rclcpp` | Jazzy | ROS 2 C++ client library |
| `nav2_msgs` | Jazzy | Navigation action messages |
| `geometry_msgs` | Jazzy | Twist, Pose messages |
| `nav_msgs` | Jazzy | Odometry, Path messages |
| `sensor_msgs` | Jazzy | IMU message type |
| `yaml-cpp` | 0.7+ | YAML file parsing |
| `Eigen3` | 3.3+ | Linear algebra |

### Development

```bash
# Ubuntu 24.04 build tools
sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  ros-jazzy-ament-cmake \
  libeigen3-dev \
  libyaml-cpp-dev
```

---

## Building from Source

### Standard Build

```bash
cd ~/ros2_ws
colcon build --packages-select waypoint_follower
```

### With Verbose Output

```bash
colcon build --packages-select waypoint_follower \
  --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON
```

### With Testing

```bash
colcon build --packages-select waypoint_follower \
  --packages-up-to waypoint_follower
  
colcon test --packages-select waypoint_follower
```

---

## Contributing

Contributions welcome! Areas for improvement:

- [ ] Cubic spline interpolation for smoother paths
- [ ] MPC controller option
- [ ] Real-time OS (PREEMPT_RT) optimization
- [ ] Multi-robot coordination
- [ ] Dynamic obstacle avoidance
- [ ] Parameter optimization tools

---

## References

### Academic Papers
- Thrun, Burgard, Fox: "Probabilistic Robotics" (Kalman filtering, motion models)
- Siciliano, Khatib: "Handbook of Robotics" (path tracking control)

### ROS 2 Resources
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Navigation Stack](https://nav2.org/)
- [TF2 Transform Library](https://docs.ros.org/en/humble/p/tf2/)

### Control Theory
- Kuo, Golnaraghi: "Automatic Control Systems" (PID control design)
- Bishop: "Modern Control Systems" (state estimation)

---

## License

MIT License - See [LICENSE](LICENSE) file for details

---

## Contact & Support

**Author:** Harsh Mulodhia  
**Repository:** [GitHub - ros2_waypoint_follower](https://github.com/HarshMulodhia/ros2_waypoint_follower)  
**Issues:** [GitHub Issues](https://github.com/HarshMulodhia/ros2_waypoint_follower/issues)

---

## Changelog

### v1.0.0 (2025-12-21)
- ✅ Initial production release
- ✅ Complete path tracking and trajectory smoothing
- ✅ Kalman filter state estimation
- ✅ ROS 2 action server interface
- ✅ Physics simulator for testing
- ✅ Comprehensive documentation

### Future (v1.1.0)
- [ ] Cubic spline interpolation
- [ ] Adaptive velocity scaling
- [ ] MPC controller option
- [ ] Multi-robot support

---

**Made with ❤️ for autonomous robotics**
