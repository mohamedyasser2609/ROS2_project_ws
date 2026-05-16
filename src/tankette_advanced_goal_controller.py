#!/usr/bin/env python3
"""
Advanced Tankette Goal Controller
Features:
- Goal queue (multiple goals)
- Timeout handling
- PID control for smooth steering
- Improved path following
- Goal status tracking
- Performance monitoring
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
import math
import time
from collections import deque


class PIDController:
    """Simple PID controller for steering"""
    
    def __init__(self, kp=0.5, ki=0.1, kd=0.2, integral_max=1.0):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.integral_max = integral_max
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
    
    def update(self, error):
        """Update PID controller and return output"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0:
            return 0.0
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(-self.integral_max, min(self.integral_max, self.integral))
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.prev_error) / dt
        
        # Update for next iteration
        self.prev_error = error
        self.last_time = current_time
        
        output = p_term + i_term + d_term
        return output
    
    def reset(self):
        """Reset PID controller state"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()


class AdvancedTanketteGoalController(Node):
    """Advanced goal navigation controller with multiple features"""
    
    def __init__(self):
        super().__init__('advanced_tankette_goal_controller')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom_raw', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data_raw', self.imu_callback, 10)
        
        # Control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # =================== GOAL QUEUE FEATURE ===================
        self.goal_queue = deque()
        self.current_goal = None
        self.goal_start_time = None
        
        # =================== STATE VARIABLES ===================
        self.latest_scan = None
        self.latest_imu = None
        self.current_pose = None
        self.has_active_goal = False
        
        # =================== CONTROL PARAMETERS ===================
        # Speed and rotation
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        self.max_linear_speed = 0.5
        self.max_angular_speed = 1.0
        
        # Tolerances
        self.distance_tolerance = 0.1  # meters
        self.angle_tolerance = 0.1  # radians
        self.obstacle_distance = 0.5  # meters
        
        # =================== TIMEOUT FEATURE ===================
        self.goal_timeout = 60.0  # seconds (0 = disabled)
        self.timeout_enabled = True
        
        # =================== PID CONTROLLER ===================
        self.pid_steering = PIDController(kp=0.8, ki=0.05, kd=0.3)
        
        # =================== PERFORMANCE TRACKING ===================
        self.goals_completed = 0
        self.goals_failed = 0
        self.total_distance_traveled = 0.0
        self.last_position = None
        
        # =================== STATE MACHINE ===================
        self.STATE_IDLE = 0
        self.STATE_ROTATING = 1
        self.STATE_MOVING = 2
        self.STATE_GOAL_REACHED = 3
        
        self.current_state = self.STATE_IDLE
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Advanced Tankette Goal Controller Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Features enabled:')
        self.get_logger().info('  ✓ Goal Queue (multiple goals)')
        self.get_logger().info('  ✓ Timeout Handling (60s per goal)')
        self.get_logger().info('  ✓ PID Control (smooth steering)')
        self.get_logger().info('  ✓ Performance Tracking')
        self.get_logger().info('  ✓ Advanced State Machine')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Click "2D Goal Pose" in RViz2 to add goals')
        self.get_logger().info('Multiple goals will be queued and executed')
        self.get_logger().info('=' * 60)
    
    def goal_callback(self, msg: PoseStamped):
        """Receive goal from RViz2 and add to queue"""
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        
        # Add to queue
        self.goal_queue.append(msg.pose)
        
        self.get_logger().info(
            f'Goal added to queue: X={goal_x:.2f}m, Y={goal_y:.2f}m '
            f'(Queue size: {len(self.goal_queue)})')
        
        # If no active goal, start this one
        if not self.has_active_goal:
            self.start_next_goal()
    
    def start_next_goal(self):
        """Start navigation to the next goal in queue"""
        if self.goal_queue:
            self.current_goal = self.goal_queue.popleft()
            self.goal_start_time = time.time()
            self.has_active_goal = True
            self.pid_steering.reset()
            self.current_state = self.STATE_IDLE
            
            goal_x = self.current_goal.position.x
            goal_y = self.current_goal.position.y
            queue_size = len(self.goal_queue)
            
            self.get_logger().info(
                f'📍 Starting navigation to: X={goal_x:.2f}m, Y={goal_y:.2f}m')
            self.get_logger().info(f'   Remaining goals in queue: {queue_size}')
        else:
            self.has_active_goal = False
            self.current_goal = None
    
    def odom_callback(self, msg: Odometry):
        """Update current pose and track distance traveled"""
        if self.current_pose is not None:
            # Calculate distance traveled
            prev_x = self.current_pose.position.x
            prev_y = self.current_pose.position.y
            
            curr_x = msg.pose.pose.position.x
            curr_y = msg.pose.pose.position.y
            
            distance = math.sqrt((curr_x - prev_x)**2 + (curr_y - prev_y)**2)
            self.total_distance_traveled += distance
        
        self.current_pose = msg.pose.pose
    
    def scan_callback(self, msg: LaserScan):
        """Store latest scan"""
        self.latest_scan = msg
    
    def imu_callback(self, msg: Imu):
        """Store latest IMU data"""
        self.latest_imu = msg
    
    def get_yaw_from_quaternion(self, quat):
        """Extract yaw from quaternion"""
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
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
        """Check for obstacles in front"""
        if not self.latest_scan:
            return False
        
        front_idx = len(self.latest_scan.ranges) // 2
        search_range = len(self.latest_scan.ranges) // 12
        
        front_ranges = self.latest_scan.ranges[
            max(0, front_idx - search_range):
            min(len(self.latest_scan.ranges), front_idx + search_range)
        ]
        
        if front_ranges:
            min_distance = min(front_ranges)
            return min_distance < self.obstacle_distance
        
        return False
    
    def check_timeout(self):
        """Check if goal has exceeded timeout"""
        if not self.timeout_enabled or self.goal_start_time is None:
            return False
        
        elapsed_time = time.time() - self.goal_start_time
        return elapsed_time > self.goal_timeout
    
    def navigate_to_goal(self):
        """Navigate to current goal with advanced control"""
        if not self.current_pose or not self.current_goal:
            return Twist()
        
        # Current state
        curr_x = self.current_pose.position.x
        curr_y = self.current_pose.position.y
        curr_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        # Goal state
        goal_x = self.current_goal.position.x
        goal_y = self.current_goal.position.y
        
        # Calculate distance and angle
        dx = goal_x - curr_x
        dy = goal_y - curr_y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_goal - curr_yaw)
        
        # Check timeout
        if self.check_timeout():
            self.get_logger().warn(
                f'⏱️  Goal timeout! ({self.goal_timeout}s exceeded)')
            self.goals_failed += 1
            self.start_next_goal()
            return Twist()
        
        # Check obstacle
        if self.check_obstacle_ahead():
            self.get_logger().warn('🚧 Obstacle detected! Stopping...')
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            return twist
        
        # Goal reached
        if distance < self.distance_tolerance:
            self.get_logger().info('🎯 Goal reached!')
            self.goals_completed += 1
            self.start_next_goal()
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            return twist
        
        twist = Twist()
        
        # State machine for navigation
        if abs(angle_error) > self.angle_tolerance:
            # STATE: ROTATING - align heading
            self.current_state = self.STATE_ROTATING
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
        else:
            # STATE: MOVING - move towards goal
            self.current_state = self.STATE_MOVING
            
            # Use PID control for smooth steering
            pid_output = self.pid_steering.update(angle_error)
            
            # Clamp angular velocity
            twist.angular.z = max(-self.max_angular_speed, 
                                 min(self.max_angular_speed, pid_output))
            
            # Adaptive linear speed based on distance and angle error
            # Slow down when close to goal or need to turn sharply
            distance_factor = min(1.0, distance / 2.0)
            angle_factor = 1.0 - abs(angle_error) / math.pi
            
            twist.linear.x = self.linear_speed * distance_factor * angle_factor
        
        # Log status occasionally
        if int(time.time() * 10) % 20 == 0:  # Every 2 seconds
            elapsed = time.time() - self.goal_start_time
            self.get_logger().info(
                f'Distance: {distance:.2f}m | '
                f'Angle error: {math.degrees(angle_error):.1f}° | '
                f'Time: {elapsed:.1f}s | '
                f'State: {self._get_state_name()}')
        
        return twist
    
    def _get_state_name(self):
        """Get human-readable state name"""
        states = {
            0: 'IDLE',
            1: 'ROTATING',
            2: 'MOVING',
            3: 'GOAL_REACHED'
        }
        return states.get(self.current_state, 'UNKNOWN')
    
    def control_loop(self):
        """Main control loop at 10Hz"""
        twist = Twist()
        
        if self.has_active_goal:
            # Navigate to goal
            twist = self.navigate_to_goal()
        else:
            # Idle - no goal
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist)
    
    def print_statistics(self):
        """Print performance statistics"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('📊 STATISTICS')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Goals Completed: {self.goals_completed}')
        self.get_logger().info(f'Goals Failed: {self.goals_failed}')
        self.get_logger().info(f'Total Distance Traveled: {self.total_distance_traveled:.2f}m')
        self.get_logger().info(f'Success Rate: {self._calculate_success_rate()}%')
        self.get_logger().info('=' * 60)
    
    def _calculate_success_rate(self):
        """Calculate goal success rate"""
        total = self.goals_completed + self.goals_failed
        if total == 0:
            return 0.0
        return round((self.goals_completed / total) * 100, 1)
    
    def destroy_node(self):
        """Cleanup when shutting down"""
        self.print_statistics()
        super().destroy_node()


def main():
    rclpy.init()
    controller = AdvancedTanketteGoalController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down controller...')
    finally:
        controller.print_statistics()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
