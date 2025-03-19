/*
 * Creates an example node to drive the vehicle forward in the CARLA simulation.
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

int main(int argc, char *argv[])
{
  // Initialize ROS and create a new node
  ros::init(argc, argv, "example_node");
  ros::NodeHandle nh;

  // Set up publishers
  ros::Publisher brake_pub = nh.advertise<std_msgs::Float64>("brake_command", 1);
  ros::Publisher gear_pub = nh.advertise<std_msgs::String>("gear_command", 1);
  ros::Publisher steering_pub = nh.advertise<std_msgs::Float64>("steering_command", 1);
  ros::Publisher throttle_pub = nh.advertise<std_msgs::Float64>("throttle_command", 1);

  // Wait for publishers to initialize
  sleep(1.0);

  // Create control messages
  std_msgs::Float64 brake_msg;
  std_msgs::String gear_msg;
  std_msgs::Float64 steering_msg;
  std_msgs::Float64 throttle_msg;

  // Set brake power to 0 and publish brake message
  brake_msg.data = 0.0;
  brake_pub.publish(brake_msg);

  // Set gear to forward and publish gear message
  gear_msg.data = "forward";
  gear_pub.publish(gear_msg);

  // Set steering position and publish steering message
  steering_msg.data = 0.0;
  steering_pub.publish(steering_msg);

  // Set throttle to 0.3 and publish throttle message
  throttle_msg.data = 0.3;
  throttle_pub.publish(throttle_msg);

  ROS_INFO_STREAM("Test control messages have been published from C++. Vehicle should be moving!");

  sleep(5.0);
  
  return 0;
}
