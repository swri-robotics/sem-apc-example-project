#!/usr/bin/env python3

"""
Creates an example node to drive the vehicle forward in the CARLA simulation.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float64

import time

def main(args=None):
    
    # Initialize ROS and create a new node
    rclpy.init(args=args)
    example_node = Node('example_node')
    
    # Set up publishers
    brake_pub_ = example_node.create_publisher(Float64, 'brake_command', 1)
    gear_pub_ = example_node.create_publisher(String, 'gear_command', 1)
    steering_pub_ = example_node.create_publisher(Float64, 'steering_command', 1)
    throttle_pub_ = example_node.create_publisher(Float64, 'throttle_command', 1)
    
    # Create control messages
    brake_msg = Float64()
    gear_msg = String()
    steering_msg = Float64()
    throttle_msg = Float64()
    
    # Set brake power to 0 and publish brake message
    brake_msg.data = 0.0
    brake_pub_.publish(brake_msg)
    
    # Set gear to forward and publish gear message
    gear_msg.data = 'forward'
    gear_pub_.publish(gear_msg)
    
    # Set steering position and publish steering message
    steering_msg.data = 0.0
    steering_pub_.publish(steering_msg)
    
    # Set throttle to 0.3 and publish throttle message
    throttle_msg.data = 0.3
    throttle_pub_.publish(throttle_msg)
    
    example_node.get_logger().info('Test control messages have been published from python. Vehicle should be moving!')
    
    time.sleep(5.0)
    
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
