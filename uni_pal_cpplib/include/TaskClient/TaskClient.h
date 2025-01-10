#ifndef TASKCLIENT_H
#define TASKCLIENT_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "uni_pal_msgs/msg/robot_static_info.hpp"
#include "uni_pal_msgs/msg/robot_dynamic_info.hpp"
#include "uni_pal_msgs/srv/execute_task.hpp"
#include "moveit/task_constructor/task.h"
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>

enum class RobotMovement : uint32_t {
    Homing = 0,
    Pick = 100,
    Place = 200,
    Demo = 999,
};

inline RobotMovement toRobotMovement(uint32_t value) {
    switch (value) {
        case 0: return RobotMovement::Homing;
        case 100: return RobotMovement::Pick;
        case 200: return RobotMovement::Place;
        case 999: return RobotMovement::Demo;
        // Add other cases as needed
        default: throw std::invalid_argument("Invalid task_nr value");
    }
}

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
    // Service server
    void execute_task_(std::shared_ptr<uni_pal_msgs::srv::ExecuteTask::Request>,
                      std::shared_ptr<uni_pal_msgs::srv::ExecuteTask::Response>);
    rclcpp::Service<uni_pal_msgs::srv::ExecuteTask>::SharedPtr execute_task_srv_;
    // MoveIt Task Constructor
    void do_task_(RobotMovement);
    moveit::task_constructor::Task create_demo_task_();
    moveit::task_constructor::Task create_homing_task_();
    moveit::task_constructor::Task task_;
};

#endif