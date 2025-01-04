#ifndef SCENECLEINT_H
#define SCENECLEINT_H

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "uni_pal_msgs/srv/get_pal_params.hpp"
#include "uni_pal_msgs/srv/empty.hpp"
#include "uni_pal_msgs/srv/get_published_transforms.hpp"
#include "uni_pal_msgs/srv/set_published_transforms.hpp"
#include "uni_pal_msgs/msg/pal_params.hpp"

class SceneClient : public rclcpp::Node
{
  public:
    SceneClient();

  private:
    // Publishers
    void publish_messages_();
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<uni_pal_msgs::msg::PalParams>::SharedPtr pal_params_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    // Service Servers
    void get_pal_params_(std::shared_ptr<uni_pal_msgs::srv::Empty::Request>,
                         std::shared_ptr<uni_pal_msgs::srv::Empty::Response>);
    void get_published_transforms_(std::shared_ptr<uni_pal_msgs::srv::GetPublishedTransforms::Request>,
                                  std::shared_ptr<uni_pal_msgs::srv::GetPublishedTransforms::Response>);
    void set_published_transforms_(std::shared_ptr<uni_pal_msgs::srv::SetPublishedTransforms::Request>,
                                  std::shared_ptr<uni_pal_msgs::srv::SetPublishedTransforms::Response>);
    rclcpp::Service<uni_pal_msgs::srv::Empty>::SharedPtr get_pal_params_srv_;
    rclcpp::Service<uni_pal_msgs::srv::GetPublishedTransforms>::SharedPtr get_published_transforms_srv;
    rclcpp::Service<uni_pal_msgs::srv::SetPublishedTransforms>::SharedPtr set_published_transforms_srv;
    // Service Clients
    void pal_params_sent_service_(rclcpp::Client<uni_pal_msgs::srv::GetPalParams>::SharedFuture);
    rclcpp::Client<uni_pal_msgs::srv::GetPalParams>::SharedPtr pal_params_client_;
    uni_pal_msgs::srv::GetPalParams_Response pal_params_response_;
    bool got_pal_params_;
    // Variables of the class
    std::vector<geometry_msgs::msg::TransformStamped> published_transforms_;
    void TODELETE_init_transforms();
};
#endif