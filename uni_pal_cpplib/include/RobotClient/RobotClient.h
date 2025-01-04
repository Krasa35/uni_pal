#ifndef ROBOTCLIENT_H
#define ROBOTCLIENT_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "uni_pal_msgs/msg/robot_static_info.hpp"
#include "uni_pal_msgs/msg/robot_dynamic_info.hpp"
#include "uni_pal_msgs/msg/robot_specific.hpp"

class RobotClient : public rclcpp::Node
{
  public:
    RobotClient();

  private:
    // Publishers
    void publish_messages_();
    rclcpp::Publisher<uni_pal_msgs::msg::RobotStaticInfo>::SharedPtr static_info_publisher_;
    rclcpp::Publisher<uni_pal_msgs::msg::RobotDynamicInfo>::SharedPtr dynamic_info_publisher_;
    rclcpp::TimerBase::SharedPtr publish_messages_timer_;
    size_t pubished_msgs_count_;
    uni_pal_msgs::msg::RobotStaticInfo static_message_;
    uni_pal_msgs::msg::RobotDynamicInfo dynamic_message_;
    // Subscribers
    void robot_specific_subscriber_callback_(const uni_pal_msgs::msg::RobotSpecific&);
    rclcpp::Subscription<uni_pal_msgs::msg::RobotSpecific>::SharedPtr robot_specific_subsciber_;
};
#endif