#include "RobotClient/RobotClient.h"

RobotClient::RobotClient() : Node("robot_client"), pubished_msgs_count_(0)
{
  RCLCPP_INFO(this->get_logger(), "robot_client node has been started.");
  // Create publishers & publish
  dynamic_info_publisher_ = this->create_publisher<uni_pal_msgs::msg::RobotDynamicInfo>("/robot_client/dynamic_info", 10);
  static_info_publisher_ = this->create_publisher<uni_pal_msgs::msg::RobotStaticInfo>("/robot_client/static_info", 10);
  publish_messages_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&RobotClient::publish_messages_, this));
  // Create subscribers
  robot_specific_subsciber_ = this->create_subscription<uni_pal_msgs::msg::RobotSpecific>(
    "/robot_specific", 10, std::bind(&RobotClient::robot_specific_subscriber_callback_, this, std::placeholders::_1));
}

// Publishers
void RobotClient::publish_messages_()
{
  dynamic_info_publisher_->publish(dynamic_message_);
  static_info_publisher_->publish(static_message_);
  pubished_msgs_count_++;
}

// Subscribers
void RobotClient::robot_specific_subscriber_callback_(const uni_pal_msgs::msg::RobotSpecific& msg)
{
  dynamic_message_.analog = msg.analog;
  dynamic_message_.digital = msg.digital;
  return;
}
