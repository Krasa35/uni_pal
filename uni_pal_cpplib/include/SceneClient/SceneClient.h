#ifndef SCENECLEINT_H
#define SCENECLEINT_H

#include <regex>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/buffer.h"
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/create_timer_ros.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "uni_pal_msgs/srv/get_pal_params.hpp"
#include "uni_pal_msgs/srv/empty.hpp"
#include "uni_pal_msgs/srv/get_published_transforms.hpp"
#include "uni_pal_msgs/srv/set_published_transforms.hpp"
#include "uni_pal_msgs/srv/set_pallet_side.hpp"
#include "uni_pal_msgs/msg/pal_params.hpp"
#include "uni_pal_msgs/msg/pallet_counters.hpp"
#include "uni_pal_msgs/msg/counters.hpp"
#include "uni_pal_msgs/msg/robot_static_info.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

class SceneClient : public rclcpp::Node
{
  public:
    SceneClient();

  private:
    // Publishers
    void publish_messages_();
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<uni_pal_msgs::msg::PalParams>::SharedPtr pal_params_publisher_;
    rclcpp::Publisher<uni_pal_msgs::msg::Counters>::SharedPtr counters_publisher_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher_;
    uni_pal_msgs::msg::Counters counters_msg_;
    moveit_msgs::msg::PlanningScene planning_scene_;
    std::vector<moveit_msgs::msg::AttachedCollisionObject> collision_objects_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    // Subscribers
    void static_message_subscriber_callback_(const uni_pal_msgs::msg::RobotStaticInfo&);
    rclcpp::Subscription<uni_pal_msgs::msg::RobotStaticInfo>::SharedPtr static_message_subscriber_;
    uni_pal_msgs::msg::RobotStaticInfo static_message_;
    // Service Servers
    void get_pal_params_(std::shared_ptr<uni_pal_msgs::srv::Empty::Request>,
                         std::shared_ptr<uni_pal_msgs::srv::Empty::Response>);
    void get_published_transforms_(std::shared_ptr<uni_pal_msgs::srv::GetPublishedTransforms::Request>,
                                  std::shared_ptr<uni_pal_msgs::srv::GetPublishedTransforms::Response>);
    void set_published_transforms_(std::shared_ptr<uni_pal_msgs::srv::SetPublishedTransforms::Request>,
                                  std::shared_ptr<uni_pal_msgs::srv::SetPublishedTransforms::Response>);
    void create_box_on_pp_(std::shared_ptr<uni_pal_msgs::srv::Empty::Request>,
                           std::shared_ptr<uni_pal_msgs::srv::Empty::Response>);
    void set_pallet_side_(std::shared_ptr<uni_pal_msgs::srv::SetPalletSide::Request>,
                          std::shared_ptr<uni_pal_msgs::srv::SetPalletSide::Response>);
    rclcpp::Service<uni_pal_msgs::srv::Empty>::SharedPtr get_pal_params_srv_;
    rclcpp::Service<uni_pal_msgs::srv::GetPublishedTransforms>::SharedPtr get_published_transforms_srv;
    rclcpp::Service<uni_pal_msgs::srv::SetPublishedTransforms>::SharedPtr set_published_transforms_srv;
    rclcpp::Service<uni_pal_msgs::srv::Empty>::SharedPtr create_box_on_pp_srv;
    rclcpp::Service<uni_pal_msgs::srv::SetPalletSide>::SharedPtr set_pallet_side_srv_;
    std::vector<geometry_msgs::msg::TransformStamped> published_transforms_;
    // Service Clients
    void pal_params_sent_service_(rclcpp::Client<uni_pal_msgs::srv::GetPalParams>::SharedFuture);
    void send_tf_request_();
    rclcpp::Client<uni_pal_msgs::srv::GetPalParams>::SharedPtr pal_params_client_;
    uni_pal_msgs::srv::GetPalParams_Response pal_params_response_;
    bool got_pal_params_;
    // Other
    void increment_counters_();
    void init_pickpoint_();
    geometry_msgs::msg::TransformStamped computeConveyorFrame();
    std::vector<geometry_msgs::msg::PoseStamped> frame_points_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    bool transform_available_ = false;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
#endif