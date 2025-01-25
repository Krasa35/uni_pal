#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "uni_pal_msgs/msg/robot_static_info.hpp"
#include "uni_pal_msgs/msg/robot_dynamic_info.hpp"
#include "uni_pal_msgs/srv/execute_task.hpp"
#include "uni_pal_msgs/srv/empty.hpp"
#include "uni_pal_msgs/srv/set_frame.hpp"
#include "uni_pal_msgs/srv/get_published_transforms.hpp"
#include "uni_pal_msgs/srv/set_published_transforms.hpp"
#include "uni_pal_msgs/srv/set_pallet_side.hpp"

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
    // Service servers
    void start_controller_(const std::shared_ptr<uni_pal_msgs::srv::Empty::Request>,
                           std::shared_ptr<uni_pal_msgs::srv::Empty::Response>);
    rclcpp::Service<uni_pal_msgs::srv::Empty>::SharedPtr start_controller_srv_;
    // Service client
    rclcpp::Client<uni_pal_msgs::srv::ExecuteTask>::SharedPtr execute_task_client_;
    rclcpp::Client<uni_pal_msgs::srv::Empty>::SharedPtr create_box_on_pp_client_;
    rclcpp::Client<uni_pal_msgs::srv::Empty>::SharedPtr update_robot_static_info_client_;
    rclcpp::Client<uni_pal_msgs::srv::Empty>::SharedPtr get_pal_params_client_;
    rclcpp::Client<uni_pal_msgs::srv::GetPublishedTransforms>::SharedPtr get_published_transforms_client_;
    rclcpp::Client<uni_pal_msgs::srv::SetPublishedTransforms>::SharedPtr set_published_transforms_client_;
    rclcpp::Client<uni_pal_msgs::srv::SetFrame>::SharedPtr set_frame_client_;
    rclcpp::Client<uni_pal_msgs::srv::SetFrame>::SharedPtr set_tcp_client_;
    rclcpp::Client<uni_pal_msgs::srv::SetPalletSide>::SharedPtr set_pallet_side_client_;
    bool service_called_;
};

#endif