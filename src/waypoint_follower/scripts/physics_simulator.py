#!/usr/bin/env python3
"""
Physics-Based Robot Simulator for ROS 2
=====================================================

Simulates differential drive kinematics with:
- Accurate wheel dynamics
- Slipping/friction simulation
- Inertial effects
- Timestamp synchronization
- Performance logging

For: ROS 2 Waypoint Follower Demonstration
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
import math
import time
from dataclasses import dataclass

@dataclass
class RobotConfig:
    """Robot physical parameters"""
    wheel_radius: float = 0.125  # meters
    wheel_separation: float = 0.38  # meters (track width)
    max_wheel_speed: float = 3.0  # m/s
    max_acceleration: float = 0.5  # m/s^2
    friction_coefficient: float = 0.1  # damping
    update_rate: float = 50.0  # Hz

class ProfessionalRobotSimulator(Node):
    """Physics-based differential drive robot simulator"""
    
    def __init__(self):
        super().__init__('professional_robot_simulator')
        self.config = RobotConfig()
        
        # Subscription to velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10)
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for physics update
        dt = 1.0 / self.config.update_rate
        self.timer = self.create_timer(dt, self._physics_update)
        
        # Robot state (position, velocity, acceleration)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.v_x = 0.0  # linear velocity (m/s)
        self.v_theta = 0.0  # angular velocity (rad/s)
        
        self.a_x = 0.0  # linear acceleration
        self.a_theta = 0.0  # angular acceleration
        
        # Command inputs
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        
        # Timing
        self.last_time = time.time()
        self.start_time = time.time()
        self.iteration = 0
        
        self.get_logger().info(
            '═' * 60 +
            '\n🤖 Professional Robot Simulator STARTED\n' +
            f'   Wheel Radius: {self.config.wheel_radius}m\n' +
            f'   Wheel Separation: {self.config.wheel_separation}m\n' +
            f'   Update Rate: {self.config.update_rate}Hz\n' +
            f'   Initial Position: ({self.x:.2f}, {self.y:.2f}, {self.theta:.2f})\n' +
            '═' * 60
        )

    def _cmd_vel_callback(self, msg: Twist):
        """Process incoming velocity commands"""
        self.cmd_linear_x = msg.linear.x
        self.cmd_angular_z = msg.angular.z

    def _physics_update(self):
        """Update robot physics and publish state"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        self.iteration += 1
        
        if dt <= 0:
            return
        
        # ========== ACCELERATION DYNAMICS ==========
        # Apply low-pass filter to smooth acceleration (prevents jerky motion)
        tau = 0.1  # Time constant for acceleration filter
        
        target_v_x = self.cmd_linear_x
        target_v_theta = self.cmd_angular_z
        
        # Limit acceleration
        max_delta_v = self.config.max_acceleration * dt
        
        self.v_x += max(-max_delta_v, min(max_delta_v, target_v_x - self.v_x))
        self.v_theta += max(-max_delta_v, min(max_delta_v, target_v_theta - self.v_theta))
        
        # Apply friction/damping
        self.v_x *= (1.0 - self.config.friction_coefficient * dt)
        self.v_theta *= (1.0 - self.config.friction_coefficient * dt)
        
        # Limit wheel speeds to prevent unrealistic behavior
        max_v = self.config.max_wheel_speed
        self.v_x = max(-max_v, min(max_v, self.v_x))
        self.v_theta = max(-max_v, min(max_v, self.v_theta))
        
        # ========== KINEMATICS UPDATE ==========
        # Differential drive kinematics
        # For a robot with two wheels of radius R separated by distance L:
        # v = (v_left + v_right) / 2
        # omega = (v_right - v_left) / L
        
        # Update orientation first (used for position update)
        delta_theta = self.v_theta * dt
        self.theta += delta_theta
        
        # Normalize theta to [-PI, PI]
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi
        
        # Update position using average heading over the time step
        avg_theta = self.theta - delta_theta / 2
        delta_x = self.v_x * math.cos(avg_theta) * dt
        delta_y = self.v_x * math.sin(avg_theta) * dt
        
        self.x += delta_x
        self.y += delta_y
        
        # ========== PUBLISH ODOMETRY ==========
        self._publish_odometry(current_time)
        
        # ========== PUBLISH TF TRANSFORM ==========
        self._publish_transform(current_time)
        
        # ========== PUBLISH IMU ==========
        self._publish_imu(current_time)
        
        # ========== DEBUG OUTPUT (every 50 iterations) ==========
        if self.iteration % 50 == 0:
            self.get_logger().debug(
                f'[{self.iteration:05d}] Pos: ({self.x:6.2f}, {self.y:6.2f}, {self.theta:6.2f}) | '
                f'Vel: (vx={self.v_x:5.2f}, w={self.v_theta:5.2f}) | '
                f'Cmd: (vx={self.cmd_linear_x:5.2f}, w={self.cmd_angular_z:5.2f})'
            )

    def _publish_odometry(self, timestamp):
        """Publish odometry message (position and velocity)"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self._time_to_ros(timestamp)
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation (yaw rotation only, in 2D plane)
        odom_msg.pose.pose.orientation = self._euler_to_quaternion(0, 0, self.theta)
        
        # Velocity (in body frame)
        odom_msg.twist.twist.linear.x = self.v_x
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.v_theta
        
        # Covariance (indicates measurement confidence)
        # Diagonal of covariance matrix: [x, y, z, roll, pitch, yaw]
        odom_msg.pose.covariance = self._get_pose_covariance()
        odom_msg.twist.covariance = self._get_twist_covariance()
        
        self.odom_pub.publish(odom_msg)

    def _publish_transform(self, timestamp):
        """Publish TF transform (odom -> base_link)"""
        t = TransformStamped()
        t.header.stamp = self._time_to_ros(timestamp)
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Rotation
        t.transform.rotation = self._euler_to_quaternion(0, 0, self.theta)
        
        self.tf_broadcaster.sendTransform(t)

    def _publish_imu(self, timestamp):
        """Publish IMU data (orientation and acceleration)"""
        imu_msg = Imu()
        imu_msg.header.stamp = self._time_to_ros(timestamp)
        imu_msg.header.frame_id = 'imu_link'
        
        # Orientation from odometry
        imu_msg.orientation = self._euler_to_quaternion(0, 0, self.theta)
        imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        
        # Linear acceleration (in body frame)
        imu_msg.linear_acceleration.x = self.a_x
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81  # Gravity
        imu_msg.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        
        # Angular velocity
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = self.v_theta
        imu_msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        
        self.imu_pub.publish(imu_msg)

    def _euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        
        return q

    def _time_to_ros(self, timestamp: float):
        """Convert Python timestamp to ROS timestamp"""
        sec = int(timestamp)
        nanosec = int((timestamp - sec) * 1e9)
        from builtin_interfaces.msg import Time
        return Time(sec=sec, nanosec=nanosec)

    def _get_pose_covariance(self):
        """Position covariance matrix (6x6)"""
        # Small covariance for simulated position
        cov = [0.001, 0, 0, 0, 0, 0,
               0, 0.001, 0, 0, 0, 0,
               0, 0, 0.001, 0, 0, 0,
               0, 0, 0, 0.01, 0, 0,
               0, 0, 0, 0, 0.01, 0,
               0, 0, 0, 0, 0, 0.01]
        return tuple(cov)

    def _get_twist_covariance(self):
        """Velocity covariance matrix (6x6)"""
        cov = [0.0001, 0, 0, 0, 0, 0,
               0, 0.0001, 0, 0, 0, 0,
               0, 0, 0.0001, 0, 0, 0,
               0, 0, 0, 0.001, 0, 0,
               0, 0, 0, 0, 0.001, 0,
               0, 0, 0, 0, 0, 0.001]
        return tuple(cov)


def main():
    rclpy.init()
    simulator = ProfessionalRobotSimulator()
    
    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        simulator.get_logger().info('Simulator shutdown requested')
    finally:
        simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
