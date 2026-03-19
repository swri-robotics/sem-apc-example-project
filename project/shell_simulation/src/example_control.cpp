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

  // Set up QoS settings for publishers
  rclcpp::QoS command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

  // Set up publishers
  auto brake_pub_ = example_node->create_publisher<std_msgs::msg::Float64>("brake_command", command_qos);
  auto gear_pub_ = example_node->create_publisher<std_msgs::msg::String>("gear_command", command_qos);
  auto steering_pub_ = example_node->create_publisher<std_msgs::msg::Float64>("steering_command", command_qos);
  auto throttle_pub_ = example_node->create_publisher<std_msgs::msg::Float64>("throttle_command", command_qos);

  // Create control messages
  std_msgs::msg::Float64 brake_msg;
  std_msgs::msg::String gear_msg;
  std_msgs::msg::Float64 steering_msg;
  std_msgs::msg::Float64 throttle_msg;

  // Set brake power to 0 to allow the vehicle to move
  brake_msg.data = 0.0;

  // Set gear to forward
  gear_msg.data = "forward";

  // Set steering position to 0 for straight
  steering_msg.data = 0.0;

  // Set throttle to 0.3 to move the vehicle forward
  throttle_msg.data = 0.3;

  RCLCPP_INFO(example_node->get_logger(), "Test control messages are being published from C++.The vehicle should be moving!");

  // Publish control messages until the node is shut down
  while (rclcpp::ok())
  {
    brake_pub_->publish(brake_msg);
    gear_pub_->publish(gear_msg);
    steering_pub_->publish(steering_msg);
    throttle_pub_->publish(throttle_msg);
    sleep(0.1);
  }

  rclcpp::shutdown();
  return 0;
}