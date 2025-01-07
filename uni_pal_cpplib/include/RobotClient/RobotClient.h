#ifndef ROBOTCLIENT_H
#define ROBOTCLIENT_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include "uni_pal_description/config/"
#include "uni_pal_msgs/srv/get_config_params.hpp"
#include "uni_pal_msgs/msg/robot_static_info.hpp"
#include "uni_pal_msgs/msg/robot_dynamic_info.hpp"
#include "uni_pal_msgs/msg/robot_specific.hpp"
#include "uni_pal_msgs/msg/robot_rpy.hpp"
#include "uni_pal_msgs/msg/config_params.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

class RobotClient : public rclcpp::Node
{
  public:
    RobotClient();

  private:
    void check_topics_();
    // Publishers
    void publish_messages_();
    rclcpp::Publisher<uni_pal_msgs::msg::RobotStaticInfo>::SharedPtr static_info_publisher_;
    rclcpp::Publisher<uni_pal_msgs::msg::RobotDynamicInfo>::SharedPtr dynamic_info_publisher_;
    rclcpp::TimerBase::SharedPtr publish_messages_timer_;
    size_t pubished_msgs_count_;
    uni_pal_msgs::msg::RobotStaticInfo static_message_, static_message_published_;
    uni_pal_msgs::msg::RobotDynamicInfo dynamic_message_, dynamic_message_published_;
    // Subscribers
    void robot_specific_subscriber_callback_(const uni_pal_msgs::msg::RobotSpecific&);
    void joint_state_subscriber_callback_(const sensor_msgs::msg::JointState&);
    geometry_msgs::msg::TransformStamped lookup_transform_(const std::string&, 
                                                          const std::string&);
    rclcpp::Subscription<uni_pal_msgs::msg::RobotSpecific>::SharedPtr robot_specific_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    // Service clients
    void get_config_params_();
    void config_params_sent_service_(rclcpp::Client<uni_pal_msgs::srv::GetConfigParams>::SharedFuture);
    rclcpp::Client<uni_pal_msgs::srv::GetConfigParams>::SharedPtr config_params_client_;
    bool got_config_params_;
    // Process methods
    geometry_msgs::msg::Pose get_pose_(const geometry_msgs::msg::TransformStamped&);
    double get_distance_(const geometry_msgs::msg::TransformStamped&);
    uni_pal_msgs::msg::RobotRPY get_rpy_(const geometry_msgs::msg::TransformStamped&);
    void get_robot_static_info_();
    void get_robot_dynamic_info_();
    void process_robot_info_();
};
#endif