#include "URRepublisher/URRepublisher.h"

URRepublisher::URRepublisher() : Node("ur_republisher"), pubished_msgs_count_(0)
  {
    RCLCPP_INFO(this->get_logger(), "ur_republisher node has been started.");
    // Create publishers & publish
    republisher_ = this->create_publisher<uni_pal_msgs::msg::RobotSpecific>("/robot_specific", 10);
    republisher_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&URRepublisher::republish_messages_, this));
    // Create subscribers
    io_subsciber_ = this->create_subscription<ur_msgs::msg::IOStates>(
      "/io_and_status_controller/io_states", 10, std::bind(&URRepublisher::io_subscriber_callback_, this, std::placeholders::_1));
    message_ = uni_pal_msgs::msg::RobotSpecific();
  }

// Publishers
void URRepublisher::republish_messages_()
  {
    republisher_->publish(message_);
    pubished_msgs_count_++;
  }

//Subscribers
void URRepublisher::io_subscriber_callback_(const ur_msgs::msg::IOStates& msg)
  {
    message_.analog.in_state.resize(msg.analog_in_states.size());
    std::transform(msg.analog_in_states.begin(), msg.analog_in_states.end(), message_.analog.in_state.begin(), [](const ur_msgs::msg::Analog_<std::allocator<void>>& val) { return static_cast<bool>(val.state); });
    message_.analog.out_state.resize(msg.analog_out_states.size());
    std::transform(msg.analog_out_states.begin(), msg.analog_out_states.end(), message_.analog.out_state.begin(), [](const ur_msgs::msg::Analog_<std::allocator<void>>& val) { return static_cast<bool>(val.state); });
    message_.digital.in_state.resize(msg.digital_in_states.size());
    std::transform(msg.digital_in_states.begin(), msg.digital_in_states.end(), message_.digital.in_state.begin(), [](const ur_msgs::msg::Digital_<std::allocator<void>>& val) { return static_cast<bool>(val.state); });
    message_.digital.out_state.resize(msg.digital_out_states.size());
    std::transform(msg.digital_out_states.begin(), msg.digital_out_states.end(), message_.digital.out_state.begin(), [](const ur_msgs::msg::Digital_<std::allocator<void>>& val) { return static_cast<bool>(val.state); });
    message_.digital.flag_state.resize(msg.flag_states.size());
    std::transform(msg.flag_states.begin(), msg.flag_states.end(), message_.digital.flag_state.begin(), [](const ur_msgs::msg::Digital_<std::allocator<void>>& val) { return static_cast<bool>(val.state); });
    return;
  }