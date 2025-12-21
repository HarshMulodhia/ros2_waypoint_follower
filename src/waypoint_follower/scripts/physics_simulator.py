#!/usr/bin/env python3
"""!
@file physics_simulator.py
@brief Physics-based robot simulator for testing
@author Harsh Mulodhia
@version 1.0.0

@class PhysicsSimulator
@brief Simulates differential-drive robot dynamics

Simulates complete robot physics:
    - Kinematic model with instantaneous center of rotation (ICR)
    - Velocity saturation to prevent unrealistic speeds
    - TF broadcasting for robot pose
    - Odometry publishing at 100 Hz

**Differential Drive Model:**
    - Two independently controlled wheels
    - Wheel base: 0.5 m (configurable)
    - Wheel radius: 0.1 m (configurable)

**Circular Motion (|ω| > 0.001):**
@code
radius = v / ω
Rotate robot around instantaneous center by ω*dt
@endcode

**Straight Motion:**
@code
x += v * cos(θ) * dt
y += v * sin(θ) * dt
@endcode

Useful for:
    - Development without real hardware
    - Algorithm validation and tuning
    - Integration testing
    - Benchmarking performance

@note Publishers: /odom, TF broadcasts
@note Subscribers: /cmd_vel
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from builtin_interfaces.msg import Time


class PhysicsSimulator(Node):
    """Simulates robot physics and publishes odometry"""
    
    def __init__(self):
        """!
        Initialize simulator node
        Subscribes to: /cmd_vel (geometry_msgs/Twist)
        Publishes: /odom (nav_msgs/Odometry)
        Broadcasts: TF /odom → /base_link
        Update rate: 100 Hz (0.01s)
        """
        super().__init__('physics_robot_simulator')
        
        # Robot parameters
        self.wheel_base = 0.5  # meters between wheels
        self.wheel_radius = 0.1  # meters
        self.max_linear_vel = 1.0  # m/s
        self.max_angular_vel = 2.0  # rad/s
        
        # Simulation state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Time tracking
        self.last_update_time = self.get_clock().now()
        
        # Create subscribers and publishers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create timer for simulation loop (100 Hz)
        self.create_timer(0.01, self.simulation_step)
        
        self.get_logger().info('Physics simulator started')
    
    def cmd_vel_callback(self, msg: Twist):
        """!
        Receive velocity commands
        @param msg geometry_msgs/Twist with linear.x and angular.z
        @effects Saturates velocities to max limits
        """
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
        
        # Saturate velocities
        self.linear_vel = max(-self.max_linear_vel, 
                             min(self.linear_vel, self.max_linear_vel))
        self.angular_vel = max(-self.max_angular_vel,
                              min(self.angular_vel, self.max_angular_vel))
    
    def simulation_step(self):
        """!
        Physics simulation step
        Algorithm for differential drive:
            IF |ω| > 0.001:  # Curved motion
                radius = v_linear / ω
                icr_x = x - radius * sin(θ)
                icr_y = y + radius * cos(θ)
                Δθ = ω * dt
                Rotate [x,y] around ICR by Δθ
            ELSE:  # Straight line
                x += v_linear * cos(θ) * dt
                y += v_linear * sin(θ) * dt
        """
        # Get time delta
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        if dt <= 0:
            dt = 0.01
        self.last_update_time = current_time
        
        # Update robot pose using differential drive kinematics
        if abs(self.angular_vel) > 0.001:
            # Circular motion
            radius = self.linear_vel / self.angular_vel
            
            # Instantaneous center of rotation
            icr_x = self.x - radius * math.sin(self.theta)
            icr_y = self.y + radius * math.cos(self.theta)
            
            # Rotate around ICR
            angle_change = self.angular_vel * dt
            cos_a = math.cos(angle_change)
            sin_a = math.sin(angle_change)
            
            # New position relative to ICR
            dx = self.x - icr_x
            dy = self.y - icr_y
            
            self.x = icr_x + dx * cos_a - dy * sin_a
            self.y = icr_y + dx * sin_a + dy * cos_a
            self.theta += angle_change
        else:
            # Straight motion
            self.x += self.linear_vel * math.cos(self.theta) * dt
            self.y += self.linear_vel * math.sin(self.theta) * dt
        
        # Normalize theta
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi
        
        # Publish odometry
        self.publish_odometry(current_time)
        
        # Publish transform
        self.publish_transform(current_time)
    
    def publish_odometry(self, timestamp):
        """!
        /**
        Publish odometry message
        @note Odometry covariance set to zero (perfect simulation)
            In real systems, set appropriate covariance values
        """
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.header.stamp = timestamp.to_msg()
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (convert theta to quaternion)
        half_theta = self.theta / 2.0
        odom.pose.pose.orientation.w = math.cos(half_theta)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(half_theta)
        
        # Velocity
        odom.twist.twist.linear.x = self.linear_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.angular_vel
        
        self.odom_pub.publish(odom)
    
    def publish_transform(self, timestamp):
        """!
        Broadcast odom → base_link transformation
        @note At 100 Hz for low-latency TF lookups
        """
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Position
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Orientation
        half_theta = self.theta / 2.0
        t.transform.rotation.w = math.cos(half_theta)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(half_theta)
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    simulator = PhysicsSimulator()
    rclpy.spin(simulator)
    simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
