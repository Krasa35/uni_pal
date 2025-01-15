#ifndef TASKCLIENT_H
#define TASKCLIENT_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "uni_pal_msgs/msg/robot_static_info.hpp"
#include "uni_pal_msgs/msg/robot_dynamic_info.hpp"
#include "uni_pal_msgs/msg/pal_params.hpp"
#include "uni_pal_msgs/msg/counters.hpp"
#include "uni_pal_msgs/srv/execute_task.hpp"
#include "uni_pal_msgs/srv/get_place_pos.hpp"
#include "moveit/task_constructor/task.h"
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>

enum class RobotMovement : uint32_t {
    Homing = 0,
    EmergencyHoming = 1,
    Pick = 100,
    Place = 200,
    Demo = 999,
};

inline RobotMovement toRobotMovement(uint32_t value) {
    switch (value) {
        case 0: return RobotMovement::Homing;
        case 1: return RobotMovement::EmergencyHoming;
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
    void pallet_params_subscriber_callback_(const uni_pal_msgs::msg::PalParams&);
    void counters_subscriber_callback_(const uni_pal_msgs::msg::Counters&);
    rclcpp::Subscription<uni_pal_msgs::msg::RobotStaticInfo>::SharedPtr static_info_subscriber_;
    rclcpp::Subscription<uni_pal_msgs::msg::RobotDynamicInfo>::SharedPtr dynamic_info_subscriber_;
    rclcpp::Subscription<uni_pal_msgs::msg::PalParams>::SharedPtr pallet_params_subscriber_;
    rclcpp::Subscription<uni_pal_msgs::msg::Counters>::SharedPtr counters_subscriber_;
    uni_pal_msgs::msg::RobotStaticInfo static_message_;
    uni_pal_msgs::msg::RobotDynamicInfo dynamic_message_;
    uni_pal_msgs::msg::PalParams pallet_params_message_;
    uni_pal_msgs::msg::Counters counters_message_;
    // Service servers
    void execute_task_(std::shared_ptr<uni_pal_msgs::srv::ExecuteTask::Request>,
                      std::shared_ptr<uni_pal_msgs::srv::ExecuteTask::Response>);
    rclcpp::Service<uni_pal_msgs::srv::ExecuteTask>::SharedPtr execute_task_srv_;
    // Service clients
    void get_place_pose_(uni_pal_msgs::srv::GetPlacePos::Request request);
    void get_place_pose_sent_service(rclcpp::Client<uni_pal_msgs::srv::GetPlacePos>::SharedFuture future);
    rclcpp::Client<uni_pal_msgs::srv::GetPlacePos>::SharedPtr get_place_pose_client_;
    uni_pal_msgs::srv::GetPlacePos::Response place_pose_;
    // MoveIt Task Constructor
    bool do_task_(RobotMovement);
    moveit::task_constructor::Task create_demo_task_();
    moveit::task_constructor::Task create_homing_task_();
    moveit::task_constructor::Task create_emergency_homing_task_();
    moveit::task_constructor::Task create_pick_task_();
    moveit::task_constructor::Task create_place_task_();
    moveit::task_constructor::Task task_;
    // Other
    // moveit_msgs::msg::CollisionObject create_box_(std::string);
};

#endif