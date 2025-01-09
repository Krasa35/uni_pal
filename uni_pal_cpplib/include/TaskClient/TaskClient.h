#ifndef TASKCLIENT_H
#define TASKCLIENT_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "uni_pal_msgs/msg/robot_static_info.hpp"
#include "uni_pal_msgs/msg/robot_dynamic_info.hpp"
#include "moveit/task_constructor/task.h"
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>

class TaskClient : public rclcpp::Node
{
  public:
    TaskClient(const rclcpp::NodeOptions&);

  private:
    // Subscribers
    void static_info_subscriber_callback_(const uni_pal_msgs::msg::RobotStaticInfo&);
    void dynamic_info_subscriber_callback_(const uni_pal_msgs::msg::RobotDynamicInfo&);
    rclcpp::Subscription<uni_pal_msgs::msg::RobotStaticInfo>::SharedPtr static_info_subscriber_;
    rclcpp::Subscription<uni_pal_msgs::msg::RobotDynamicInfo>::SharedPtr dynamic_info_subscriber_;
    uni_pal_msgs::msg::RobotStaticInfo static_message_;
    uni_pal_msgs::msg::RobotDynamicInfo dynamic_message_;
    // MoveIt Task Constructor
    void do_task_();
    moveit::task_constructor::Task create_demo_task_();
    moveit::task_constructor::Task task_;
    rclcpp::TimerBase::SharedPtr timer_;
};
#endif