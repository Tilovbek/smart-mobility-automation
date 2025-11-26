#!/bin/bash
# Test script for color following node
# Creates a camera simulator that publishes images with colored objects

echo "========================================="
echo "Color Following Test Script"
echo "========================================="
echo ""
echo "This script will:"
echo "  1. Create a camera simulator"
echo "  2. Publish images with colored objects"
echo "  3. Monitor color following behavior"
echo ""
echo "Make sure the color following node is running:"
echo "  ros2 launch turtlebot3_automation color_follow.launch.py"
echo ""
echo "Available colors: red, blue, green, yellow, orange"
echo "Default target: red"
echo ""
echo "Press Ctrl+C to stop"
echo "========================================="
echo ""

# Source ROS 2 environment
source /opt/ros/foxy/setup.bash
source install/setup.bash

# Create a Python script to simulate camera with colored objects
python3.8 << 'EOF'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2
import time
import random

class ColorCameraSimulator(Node):
    def __init__(self):
        super().__init__('color_camera_simulator')
        self.image_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.timer = self.create_timer(0.5, self.publish_image)  # 2 Hz
        
        self.get_logger().info('='*60)
        self.get_logger().info('Color Camera Simulator Started')
        self.get_logger().info('Publishing to: /camera/color/image_raw')
        self.get_logger().info('Monitoring: /cmd_vel')
        self.get_logger().info('='*60)
        
        self.frame_count = 0
        self.last_cmd = None
        
        # Color definitions (BGR format for OpenCV)
        self.colors = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'yellow': (0, 255, 255),
            'orange': (0, 165, 255)
        }
        
        # Object positions and movement
        self.objects = []
        self.create_random_objects()
    
    def cmd_callback(self, msg):
        """Monitor commands from color following node"""
        self.last_cmd = msg
        self.get_logger().info(
            f'🤖 CMD: linear={msg.linear.x:.3f}, angular={msg.angular.z:.3f}',
            throttle_duration_sec=0.5
        )
    
    def create_random_objects(self):
        """Create random colored objects in the scene"""
        for _ in range(random.randint(1, 3)):  # 1-3 objects
            obj = {
                'color': random.choice(list(self.colors.keys())),
                'x': random.randint(100, 540),  # Keep away from edges
                'y': random.randint(200, 350),  # Middle third of frame
                'size': random.randint(30, 80),  # Object size
                'dx': random.randint(-3, 3),     # Movement
                'dy': random.randint(-2, 2)
            }
            self.objects.append(obj)
    
    def update_objects(self):
        """Move objects around"""
        for obj in self.objects:
            obj['x'] += obj['dx']
            obj['y'] += obj['dy']
            
            # Bounce off edges
            if obj['x'] <= obj['size'] or obj['x'] >= 640 - obj['size']:
                obj['dx'] *= -1
            if obj['y'] <= obj['size'] or obj['y'] >= 480 - obj['size']:
                obj['dy'] *= -1
    
    def create_image(self):
        """Create an image with colored objects"""
        # Start with a neutral background
        img = np.ones((480, 640, 3), dtype=np.uint8) * 180  # Light gray
        
        # Add some texture/noise
        noise = np.random.randint(-20, 20, img.shape, dtype=np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        
        # Draw colored objects
        for obj in self.objects:
            color_bgr = self.colors[obj['color']]
            center = (obj['x'], obj['y'])
            radius = obj['size']
            
            # Draw filled circle
            cv2.circle(img, center, radius, color_bgr, -1)
            
            # Add some shading for realism
            cv2.circle(img, (center[0]-radius//3, center[1]-radius//3), 
                      radius//2, tuple(max(0, c-50) for c in color_bgr), -1)
        
        return img
    
    def publish_image(self):
        """Publish camera image with colored objects"""
        # Update object positions
        self.update_objects()
        
        # Create image
        img = self.create_image()
        
        # Create ROS Image message
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.height = 480
        msg.width = 640
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = 640 * 3
        msg.data = img.tobytes()
        
        self.image_pub.publish(msg)
        self.frame_count += 1
        
        # Status update
        if self.frame_count % 10 == 0:
            obj_info = [f"{obj['color']}@{obj['x']},{obj['y']}" for obj in self.objects]
            self.get_logger().info(
                f'📹 Frame {self.frame_count} | Objects: {", ".join(obj_info)}'
            )

def main():
    rclpy.init()
    node = ColorCameraSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('')
        node.get_logger().info('='*60)
        node.get_logger().info(f'Color camera simulator stopped')
        node.get_logger().info(f'Total frames published: {node.frame_count}')
        node.get_logger().info('='*60)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF