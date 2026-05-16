#!/usr/bin/env python3
"""
Tankette Controller Script
Controls the tankette robot and reads sensor data from Gazebo
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
import math
import time


class TanketteController(Node):
    """Node to control tankette and subscribe to sensor data"""
    
    def __init__(self):
        super().__init__('tankette_controller')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers for sensor data
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data_raw', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom_raw', self.odom_callback, 10)
        
        # Create timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # State variables
        self.latest_scan = None
        self.latest_imu = None
        self.latest_odom = None
        self.control_state = 0
        self.state_timer = 0
        
        self.get_logger().info('Tankette controller initialized')
        self.get_logger().info('Publishing velocity commands to /cmd_vel')
        self.get_logger().info('Listening to /scan, /imu/data_raw, and /odom_raw topics')
    
    def scan_callback(self, msg: LaserScan):
        """Callback for LiDAR scan data"""
        self.latest_scan = msg
        
        # Example: Find minimum distance in front
        if len(msg.ranges) > 0:
            front_idx = int(len(msg.ranges) / 2)
            search_range = 30
            front_range = msg.ranges[max(0, front_idx - search_range):
                                     min(len(msg.ranges), front_idx + search_range)]
            min_distance = min(front_range) if front_range else float('inf')
            
            # Only log occasionally to avoid spam
            if self.state_timer % 10 == 0:
                self.get_logger().info(f'LiDAR - Min distance ahead: {min_distance:.2f}m')
    
    def imu_callback(self, msg: Imu):
        """Callback for IMU data"""
        self.latest_imu = msg
        
        # Log acceleration every 10 calls
        if self.state_timer % 10 == 0:
            accel = msg.linear_acceleration
            self.get_logger().info(
                f'IMU - Acceleration: X={accel.x:.2f}, Y={accel.y:.2f}, Z={accel.z:.2f} m/s²')
    
    def odom_callback(self, msg: Odometry):
        """Callback for odometry data"""
        self.latest_odom = msg
        
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        
        # Log position every 10 calls
        if self.state_timer % 10 == 0:
            self.get_logger().info(
                f'Odometry - Position: X={pos.x:.2f}, Y={pos.y:.2f}, Z={pos.z:.2f}m')
    
    def control_loop(self):
        """Main control loop - runs at 10Hz"""
        self.state_timer += 1
        
        # Create velocity command
        twist = Twist()
        
        # Simple state machine for testing
        # State 0: Move forward for 3 seconds
        # State 1: Turn left for 2 seconds
        # State 2: Move forward for 3 seconds
        # State 3: Turn right for 2 seconds
        # Repeat
        
        if self.control_state == 0:  # Move forward
            twist.linear.x = 0.5  # 0.5 m/s forward
            twist.angular.z = 0.0
            if self.state_timer > 30:  # 3 seconds at 10Hz
                self.control_state = 1
                self.state_timer = 0
                self.get_logger().info('State change: Moving forward -> Turning left')
        
        elif self.control_state == 1:  # Turn left
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # 0.5 rad/s counter-clockwise
            if self.state_timer > 20:  # 2 seconds
                self.control_state = 2
                self.state_timer = 0
                self.get_logger().info('State change: Turning left -> Moving forward')
        
        elif self.control_state == 2:  # Move forward again
            twist.linear.x = 0.5
            twist.angular.z = 0.0
            if self.state_timer > 30:
                self.control_state = 3
                self.state_timer = 0
                self.get_logger().info('State change: Moving forward -> Turning right')
        
        elif self.control_state == 3:  # Turn right
            twist.linear.x = 0.0
            twist.angular.z = -0.5  # 0.5 rad/s clockwise
            if self.state_timer > 20:
                self.control_state = 0
                self.state_timer = 0
                self.get_logger().info('State change: Turning right -> Moving forward (cycle complete)')
        
        # Publish velocity command
        self.cmd_vel_pub.publish(twist)


def main():
    rclpy.init()
    
    # Create and run the controller node
    controller = TanketteController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Controller shutting down...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
