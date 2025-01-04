#include "SceneClient/SceneClient.h"

SceneClient::SceneClient() : Node("scene_client")
  {
    RCLCPP_INFO(this->get_logger(), "scene_client node has been started.");
    // Publishers
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    pal_params_publisher_ = this->create_publisher<uni_pal_msgs::msg::PalParams>("/scene_client/pal_params", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&SceneClient::publish_messages_, this));
    // Service Servers
    get_pal_params_srv_ = this->create_service<uni_pal_msgs::srv::Empty>(
        "/scene_client/get_pal_params",std::bind(&SceneClient::get_pal_params_, this, std::placeholders::_1, std::placeholders::_2)) ;
    get_published_transforms_srv = this->create_service<uni_pal_msgs::srv::GetPublishedTransforms>(
        "/scene_client/get_published_transforms",std::bind(&SceneClient::get_published_transforms_, this, std::placeholders::_1, std::placeholders::_2)) ;
    set_published_transforms_srv = this->create_service<uni_pal_msgs::srv::SetPublishedTransforms>(
        "/scene_client/set_published_transforms",std::bind(&SceneClient::set_published_transforms_, this, std::placeholders::_1, std::placeholders::_2)) ;

    // Service Clients
    pal_params_client_ = this->create_client<uni_pal_msgs::srv::GetPalParams>("/read_json_node/get_pallet_params");
    got_pal_params_ = false;
    TODELETE_init_transforms();
  }
// Publishers
void SceneClient::publish_messages_()
  {
    if (!got_pal_params_)
    {
      return;
    }

    geometry_msgs::msg::TransformStamped t;
    for (const auto& transform : published_transforms_) {
      tf_broadcaster_->sendTransform(transform);
    }

    uni_pal_msgs::msg::PalParams msg = pal_params_response_.params;
    pal_params_publisher_->publish(msg);
  }
// Service Servers
void SceneClient::get_pal_params_(std::shared_ptr<uni_pal_msgs::srv::Empty::Request>,
                                  std::shared_ptr<uni_pal_msgs::srv::Empty::Response>)
  {
    if (!pal_params_client_->wait_for_service(std::chrono::seconds(1))) 
    {
      if (rclcpp::ok()) 
      {
        RCLCPP_ERROR(this->get_logger(),
                      "Client interrupted while waiting for service. Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service Unavailable. Waiting for Service...");
    }
    auto request = std::make_shared<uni_pal_msgs::srv::GetPalParams_Request>();
    // set request variables here, if any
    got_pal_params_ = false;
    auto result_future = pal_params_client_->async_send_request(
        request, std::bind(&SceneClient::pal_params_sent_service_, this,
                          std::placeholders::_1));
  }
void SceneClient::get_published_transforms_(std::shared_ptr<uni_pal_msgs::srv::GetPublishedTransforms::Request>,
                                           std::shared_ptr<uni_pal_msgs::srv::GetPublishedTransforms::Response> response)
  {
    response->transforms = published_transforms_;
  }
void SceneClient::set_published_transforms_(std::shared_ptr<uni_pal_msgs::srv::SetPublishedTransforms::Request> request,
                                           std::shared_ptr<uni_pal_msgs::srv::SetPublishedTransforms::Response>)
  {
    published_transforms_ = request->transforms;
  }

// Service Clients
void SceneClient::pal_params_sent_service_(rclcpp::Client<uni_pal_msgs::srv::GetPalParams>::SharedFuture future)
  { 
    auto status = future.wait_for(std::chrono::seconds(1));
    if (status == std::future_status::ready) 
    {
      pal_params_response_ = *(future.get());
      got_pal_params_ = true;
      RCLCPP_INFO(this->get_logger(), "Result: success");
    }
    else 
    {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }
// OTHER
void SceneClient::TODELETE_init_transforms()
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "pallet_left";
    t.child_frame_id = "pallet_left_robot"; //"length" : 1219,"width" : 1016,
    tf2::Vector3 v(-0.508, -0.609,  0.0);
    t.transform.translation = tf2::toMsg(v);//{-0.508, -0.609,  0.0};
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    t.transform.rotation = tf2::toMsg(q);
    published_transforms_.push_back(t);

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "pallet_right";
    t.child_frame_id = "pallet_right_robot";
    v = tf2::Vector3(-0.508, -0.609, 0.0);
    t.transform.translation = tf2::toMsg(v);
    q.setRPY(0, 0, 0);
    t.transform.rotation = tf2::toMsg(q);
    published_transforms_.push_back(t);
  }