#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "uni_pal_msgs/msg/robot_static_info.hpp"
#include "uni_pal_msgs/msg/robot_dynamic_info.hpp"
#include "uni_pal_msgs/srv/execute_task.hpp"

class Controller : public rclcpp::Node
{
  public:
    Controller();

  private:
    // Subscribers
    void static_info_subscriber_callback_(const uni_pal_msgs::msg::RobotStaticInfo&);
    void dynamic_info_subscriber_callback_(const uni_pal_msgs::msg::RobotDynamicInfo&);
    rclcpp::Subscription<uni_pal_msgs::msg::RobotStaticInfo>::SharedPtr static_info_subscriber_;
    rclcpp::Subscription<uni_pal_msgs::msg::RobotDynamicInfo>::SharedPtr dynamic_info_subscriber_;
    uni_pal_msgs::msg::RobotStaticInfo static_message_;
    uni_pal_msgs::msg::RobotDynamicInfo dynamic_message_;
    // Service client
};

#endif