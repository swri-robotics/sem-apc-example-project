/*
 * Creates an example node to drive the vehicle forward in the CARLA simulation.
 */

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include <iostream>

int main(int argc, char *argv[])
{
  // Initialize ROS and create a new node
  rclcpp::init(argc, argv);
  auto example_node = rclcpp::Node::make_shared("example_node");

  // Set up publishers
  auto brake_pub_ = example_node->create_publisher<std_msgs::msg::Float64>("brake_command", 1);
  auto gear_pub_ = example_node->create_publisher<std_msgs::msg::String>("gear_command", 1);
  auto steering_pub_ = example_node->create_publisher<std_msgs::msg::Float64>("steering_command", 1);
  auto throttle_pub_ = example_node->create_publisher<std_msgs::msg::Float64>("throttle_command", 1);

  // Create control messages
  std_msgs::msg::Float64 brake_msg;
  std_msgs::msg::String gear_msg;
  std_msgs::msg::Float64 steering_msg;
  std_msgs::msg::Float64 throttle_msg;

  // Set brake power to 0 and publish brake message
  brake_msg.data = 0.0;
  brake_pub_->publish(brake_msg);

  // Set gear to forward and publish gear message
  gear_msg.data = "forward";
  gear_pub_->publish(gear_msg);

  // Set steering position and publish steering message
  steering_msg.data = 0.0;
  steering_pub_->publish(steering_msg);

  // Set throttle to 0.3 and publish throttle message
  throttle_msg.data = 0.3;
  throttle_pub_->publish(throttle_msg);

  RCLCPP_INFO(example_node->get_logger(), "Test control messages have been published from C++. Vehicle should be moving!");

  sleep(5.0);

  rclcpp::shutdown();
  return 0;
}