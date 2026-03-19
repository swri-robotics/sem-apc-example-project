#!/usr/bin/env python3

"""
Creates an example node to drive the vehicle forward in the CARLA simulation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import String
from std_msgs.msg import Float64

import time

def main(args=None):
    
    # Initialize ROS and create a new node
    rclpy.init(args=args)
    example_node = Node('example_node')
        
    # Set up QoS settings for publishers
    command_qos = QoSProfile(
        reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        depth=1
    )
    
    # Set up publishers
    brake_pub_ = example_node.create_publisher(Float64, 'brake_command', command_qos)
    gear_pub_ = example_node.create_publisher(String, 'gear_command', command_qos)
    steering_pub_ = example_node.create_publisher(Float64, 'steering_command', command_qos)
    throttle_pub_ = example_node.create_publisher(Float64, 'throttle_command', command_qos)
    
    # Create control messages
    brake_msg = Float64()
    gear_msg = String()
    steering_msg = Float64()
    throttle_msg = Float64()
    
    # Set brake power to 0 to allow the vehicle to move
    brake_msg.data = 0.0
    
    # Set gear to forward 
    gear_msg.data = 'forward'
    
    # Set steering position to 0 for straight
    steering_msg.data = 0.0
    
    # Set throttle to 0.3 to move the vehicle forward
    throttle_msg.data = 0.3
    
    example_node.get_logger().info('Test control messages are being published from python. The vehicle should be moving!')
    
    # Publish control messages until the node is shut down
    while rclpy.ok():
        brake_pub_.publish(brake_msg)
        gear_pub_.publish(gear_msg)
        steering_pub_.publish(steering_msg)
        throttle_pub_.publish(throttle_msg)
        time.sleep(0.1)
        
    rclpy.shutdown()
    
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
