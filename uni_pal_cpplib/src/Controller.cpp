#include "Controller/Controller.h"

#include "TaskClient/TaskClient.h"

Controller::Controller() : Node("task_client")
{
  // Subscribers
  static_info_subscriber_ = this->create_subscription<uni_pal_msgs::msg::RobotStaticInfo>(
      "/robot_client/static_info", 10, std::bind(&Controller::static_info_subscriber_callback_, this, std::placeholders::_1));
  dynamic_info_subscriber_ = this->create_subscription<uni_pal_msgs::msg::RobotDynamicInfo>(
      "/robot_client/dynamic_info", 10, std::bind(&Controller::dynamic_info_subscriber_callback_, this, std::placeholders::_1));
  // Service client
}

// Subscribers
void Controller::static_info_subscriber_callback_(const uni_pal_msgs::msg::RobotStaticInfo &msg)
{
  static_message_ = msg;
}

void Controller::dynamic_info_subscriber_callback_(const uni_pal_msgs::msg::RobotDynamicInfo &msg)
{
  dynamic_message_ = msg;
}

// Service client
