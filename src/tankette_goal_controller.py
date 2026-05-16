#!/usr/bin/env python3
"""
Tankette Goal Pose Controller
Subscribes to RViz2 2D Goal Pose and navigates the tankette to the goal
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
import math


class TanketteGoalController(Node):
    """Navigate tankette to goal pose from RViz2"""
    
    def __init__(self):
        super().__init__('tankette_goal_controller')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers for sensor data
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom_raw', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data_raw', self.imu_callback, 10)
        
        # Create timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # State variables
        self.latest_scan = None
        self.latest_imu = None
        self.current_pose = None
        self.goal_pose = None
        self.has_goal = False
        
        # Control parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        self.distance_tolerance = 0.1  # meters
        self.angle_tolerance = 0.1  # radians (~5.7 degrees)
        self.obstacle_distance = 0.5  # meters
        
        self.get_logger().info('Tankette Goal Controller initialized')
        self.get_logger().info('Click "2D Goal Pose" in RViz2 to set a goal')
        self.get_logger().info('Listening to /goal_pose topic')
        self.get_logger().info('Robot will navigate to the selected goal')
    
    def goal_callback(self, msg: PoseStamped):
        """Callback for RViz2 goal pose"""
        self.goal_pose = msg.pose
        self.has_goal = True
        
        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y
        
        self.get_logger().info(
            f'New goal received: X={goal_x:.2f}m, Y={goal_y:.2f}m')
    
    def odom_callback(self, msg: Odometry):
        """Callback for odometry data"""
        self.current_pose = msg.pose.pose
    
    def scan_callback(self, msg: LaserScan):
        """Callback for LiDAR scan data"""
        self.latest_scan = msg
    
    def imu_callback(self, msg: Imu):
        """Callback for IMU data"""
        self.latest_imu = msg
    
    def get_yaw_from_quaternion(self, quat):
        """Extract yaw angle from quaternion"""
        # For 2D, we only care about rotation around Z axis
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        
        # Calculate yaw using atan2
        yaw = math.atan2(2.0 * (w * z + x * y),
                         1.0 - 2.0 * (y * y + z * z))
        return yaw
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def check_obstacle_ahead(self):
        """Check if there's an obstacle in front"""
        if not self.latest_scan:
            return False
        
        # Check front region (center ±30 degrees)
        front_idx = len(self.latest_scan.ranges) // 2
        search_range = len(self.latest_scan.ranges) // 12  # ±30 degrees
        
        front_ranges = self.latest_scan.ranges[
            max(0, front_idx - search_range):
            min(len(self.latest_scan.ranges), front_idx + search_range)
        ]
        
        if front_ranges:
            min_distance = min(front_ranges)
            return min_distance < self.obstacle_distance
        
        return False
    
    def navigate_to_goal(self):
        """Calculate velocity commands to reach goal"""
        if not self.current_pose or not self.goal_pose:
            return None
        
        # Current position and orientation
        curr_x = self.current_pose.position.x
        curr_y = self.current_pose.position.y
        curr_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        # Goal position
        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y
        
        # Calculate distance to goal
        dx = goal_x - curr_x
        dy = goal_y - curr_y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Calculate angle to goal
        angle_to_goal = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_goal - curr_yaw)
        
        # Create velocity command
        twist = Twist()
        
        # Check for obstacles
        if self.check_obstacle_ahead():
            self.get_logger().warn('Obstacle detected! Stopping...')
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            return twist
        
        # Goal reached?
        if distance < self.distance_tolerance:
            self.get_logger().info('Goal reached!')
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.has_goal = False
            return twist
        
        # Heading alignment phase
        if abs(angle_error) > self.angle_tolerance:
            # Rotate towards goal
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
            return twist
        
        # Move towards goal
        twist.linear.x = self.linear_speed
        twist.angular.z = angle_error * 0.5  # Proportional control for steering
        
        # Log progress occasionally
        if int((curr_x + curr_y) * 100) % 10 == 0:  # Sparse logging
            self.get_logger().info(
                f'Distance to goal: {distance:.2f}m, '
                f'Angle error: {math.degrees(angle_error):.1f}°')
        
        return twist
    
    def control_loop(self):
        """Main control loop"""
        twist = Twist()
        
        if self.has_goal:
            # Navigate to goal
            cmd = self.navigate_to_goal()
            if cmd:
                twist = cmd
        else:
            # No goal, stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        # Publish velocity command
        self.cmd_vel_pub.publish(twist)


def main():
    rclpy.init()
    controller = TanketteGoalController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Controller shutting down...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
