#!/usr/bin/env python3
"""
SLAM Map Bootstrap Controller
Moves the robot in a scanning pattern to generate an initial SLAM map
Then allows switching to Nav2 goal navigation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import time
import math


class MapBootstrapController(Node):
    """Move robot to generate initial map for SLAM"""
    
    def __init__(self):
        super().__init__('map_bootstrap_controller')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribe to map to detect when it's ready
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        # Control timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # State machine
        self.STATE_SPINNING = 0
        self.STATE_FORWARD = 1
        self.STATE_SPINNING_AGAIN = 2
        self.STATE_MAP_READY = 3
        self.STATE_IDLE = 4
        
        self.current_state = self.STATE_SPINNING
        self.state_timer = 0
        self.map_received = False
        self.map_size = 0
        
        # Parameters
        self.forward_speed = 0.3  # m/s
        self.spin_speed = 0.8  # rad/s
        self.spin_duration = 60  # 6 seconds at 10Hz
        self.forward_duration = 40  # 4 seconds
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('SLAM Map Bootstrap Controller')
        self.get_logger().info('=' * 70)
        self.get_logger().info('This controller will:')
        self.get_logger().info('  1. Spin in place to scan environment (6 seconds)')
        self.get_logger().info('  2. Move forward while scanning (4 seconds)')
        self.get_logger().info('  3. Spin again to complete map (6 seconds)')
        self.get_logger().info('  4. Once map is ready, switch to idle mode')
        self.get_logger().info('  5. You can then use Nav2 goals!')
        self.get_logger().info('=' * 70)
        self.get_logger().info('Starting exploration sequence...')
        self.get_logger().info('=' * 70)
    
    def map_callback(self, msg: OccupancyGrid):
        """Monitor map size"""
        if msg.data:
            self.map_received = True
            # Count non-zero cells to estimate map quality
            non_unknown = sum(1 for cell in msg.data if cell != -1)
            self.map_size = non_unknown
    
    def control_loop(self):
        """Main control loop"""
        self.state_timer += 1
        twist = Twist()
        
        # State machine for exploration
        if self.current_state == self.STATE_SPINNING:
            # Phase 1: Spin in place to scan
            twist.linear.x = 0.0
            twist.angular.z = self.spin_speed
            
            if self.state_timer % 10 == 0:
                elapsed = self.state_timer / 10.0
                self.get_logger().info(f'[Spinning] {elapsed:.1f}s - Scanning environment...')
            
            if self.state_timer >= self.spin_duration:
                self.state_timer = 0
                self.current_state = self.STATE_FORWARD
                self.get_logger().info('[Moving Forward] Starting to move and scan...')
        
        elif self.current_state == self.STATE_FORWARD:
            # Phase 2: Move forward while scanning
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.3  # Slight turn while moving
            
            if self.state_timer % 10 == 0:
                elapsed = self.state_timer / 10.0
                self.get_logger().info(f'[Moving] {elapsed:.1f}s - {self.forward_speed} m/s')
            
            if self.state_timer >= self.forward_duration:
                self.state_timer = 0
                self.current_state = self.STATE_SPINNING_AGAIN
                self.get_logger().info('[Spinning Again] Second scan phase...')
        
        elif self.current_state == self.STATE_SPINNING_AGAIN:
            # Phase 3: Spin again in opposite direction
            twist.linear.x = 0.0
            twist.angular.z = -self.spin_speed
            
            if self.state_timer % 10 == 0:
                elapsed = self.state_timer / 10.0
                self.get_logger().info(f'[Spinning Opposite] {elapsed:.1f}s')
            
            if self.state_timer >= self.spin_duration:
                self.state_timer = 0
                self.current_state = self.STATE_MAP_READY
                self.get_logger().info('✅ Exploration complete!')
                self.get_logger().info('=' * 70)
        
        elif self.current_state == self.STATE_MAP_READY:
            # Check if map is actually ready
            if self.map_received and self.map_size > 100:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.current_state = self.STATE_IDLE
                
                self.get_logger().info('🎉 MAP READY!')
                self.get_logger().info(f'📊 Map size: {self.map_size} cells')
                self.get_logger().info('=' * 70)
                self.get_logger().info('You can now:')
                self.get_logger().info('  1. Open RViz2')
                self.get_logger().info('  2. Click "Nav2 Goal" button')
                self.get_logger().info('  3. Click on the map to send goals')
                self.get_logger().info('  4. Robot will autonomously navigate!')
                self.get_logger().info('=' * 70)
            else:
                # Map not ready yet, wait
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                
                if self.state_timer % 20 == 0:
                    self.get_logger().info(f'Waiting for map... (size: {self.map_size})')
                    self.get_logger().info('(This should take 10-30 seconds)')
        
        elif self.current_state == self.STATE_IDLE:
            # Idle - no movement
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
            # Periodically check if map is still good
            if self.state_timer % 100 == 0:
                self.get_logger().info(f'✅ Idle. Ready for Nav2 goals. Map size: {self.map_size}')
        
        # Publish velocity command
        self.cmd_vel_pub.publish(twist)


def main():
    rclpy.init()
    controller = MapBootstrapController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Bootstrap controller stopped')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
