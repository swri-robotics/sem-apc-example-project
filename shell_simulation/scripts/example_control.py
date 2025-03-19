#!/usr/bin/env python3

"""
Creates an example node to drive the vehicle forward in the CARLA simulation.
"""

import rospy

from std_msgs.msg import String
from std_msgs.msg import Float64

import time

def main(args=None):
    
    # Initialize ROS and create a new node
    rospy.init_node('example_node')
    
    # Set up publishers
    brake_pub = rospy.Publisher('/brake_command', Float64, queue_size=1)
    gear_pub = rospy.Publisher('/gear_command', String, queue_size=1)
    steering_pub = rospy.Publisher('/steering_command', Float64, queue_size=1)
    throttle_pub = rospy.Publisher('/throttle_command', Float64, queue_size=1)

    # Wait for publishers to initialize
    time.sleep(1.0)

    # Create control messages
    brake_msg = Float64()
    gear_msg = String()
    steering_msg = Float64()
    throttle_msg = Float64()
    
    # Set brake power to 0 and publish brake message
    brake_msg.data = 0.0
    brake_pub.publish(brake_msg)
    
    # Set gear to forward and publish gear message
    gear_msg.data = 'forward'
    gear_pub.publish(gear_msg)
    
    # Set steering position and publish steering message
    steering_msg.data = 0.0
    steering_pub.publish(steering_msg)
    
    # Set throttle to 0.3 and publish throttle message
    throttle_msg.data = 0.3
    throttle_pub.publish(throttle_msg)

    rospy.loginfo('Test control messages have been published from python. Vehicle should be moving!')
        
    time.sleep(5.0)
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
