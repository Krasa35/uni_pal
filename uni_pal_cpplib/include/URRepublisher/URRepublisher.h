#ifndef URREPUBLISHER_H
#define URREPUBLISHER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "uni_pal_msgs/msg/robot_specific.hpp"
#include "ur_msgs/msg/io_states.hpp"

class URRepublisher : public rclcpp::Node
{
  public:
    URRepublisher();

  private:
    // Publishers
    void republish_messages_();
    rclcpp::Publisher<uni_pal_msgs::msg::RobotSpecific>::SharedPtr republisher_;
    rclcpp::TimerBase::SharedPtr republisher_timer_;
    size_t pubished_msgs_count_;
    uni_pal_msgs::msg::RobotSpecific message_;
    // Subscribers
    void io_subscriber_callback_(const ur_msgs::msg::IOStates&);
    rclcpp::Subscription<ur_msgs::msg::IOStates>::SharedPtr io_subsciber_;

};
#endif