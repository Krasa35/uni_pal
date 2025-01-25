#include "Controller/Controller.h"

Controller::Controller() : Node("controller")
{
  // Subscribers
  static_info_subscriber_ = this->create_subscription<uni_pal_msgs::msg::RobotStaticInfo>(
      "/robot_client/static_info", 10, std::bind(&Controller::static_info_subscriber_callback_, this, std::placeholders::_1));
  dynamic_info_subscriber_ = this->create_subscription<uni_pal_msgs::msg::RobotDynamicInfo>(
      "/robot_client/dynamic_info", 10, std::bind(&Controller::dynamic_info_subscriber_callback_, this, std::placeholders::_1));
  // Service servers
  start_controller_srv_ = this->create_service<uni_pal_msgs::srv::Empty>(
      "/controller/start", std::bind(&Controller::start_controller_, this, std::placeholders::_1, std::placeholders::_2));
  // Service client
  execute_task_client_ = this->create_client<uni_pal_msgs::srv::ExecuteTask>("/task_client/execute_task");
  create_box_on_pp_client_ = this->create_client<uni_pal_msgs::srv::Empty>("/scene_client/create_box_on_pp");
  update_robot_static_info_client_ = this->create_client<uni_pal_msgs::srv::Empty>("/robot_client/update_robot_static_info");
  get_pal_params_client_ = this->create_client<uni_pal_msgs::srv::Empty>("/scene_client/get_pal_params");
  get_published_transforms_client_ = this->create_client<uni_pal_msgs::srv::GetPublishedTransforms>("/scene_client/get_published_transforms");
  set_published_transforms_client_ = this->create_client<uni_pal_msgs::srv::SetPublishedTransforms>("/scene_client/set_published_transforms");
  set_frame_client_ = this->create_client<uni_pal_msgs::srv::SetFrame>("/robot_client/set_frame");
  set_tcp_client_ = this->create_client<uni_pal_msgs::srv::SetFrame>("/robot_client/set_tcp");
  set_pallet_side_client_ = this->create_client<uni_pal_msgs::srv::SetPalletSide>("/scene_client/set_pallet_side");
  service_called_ = false;
}

// Subscribers
void Controller::static_info_subscriber_callback_(const uni_pal_msgs::msg::RobotStaticInfo &msg)
{
  static_message_ = msg;
}

void Controller::dynamic_info_subscriber_callback_(const uni_pal_msgs::msg::RobotDynamicInfo &msg)
{
  dynamic_message_ = msg;
}

//Service servers
void Controller::start_controller_(const std::shared_ptr<uni_pal_msgs::srv::Empty::Request>,
                                   std::shared_ptr<uni_pal_msgs::srv::Empty::Response>)
{
  RCLCPP_INFO(this->get_logger(), "Controller started");
  // if (!create_box_on_pp_client_->wait_for_service(std::chrono::seconds(1)) ||
  //     !execute_task_client_->wait_for_service(std::chrono::seconds(1)) ||
  //     !update_robot_static_info_client_->wait_for_service(std::chrono::seconds(1)) ||
  //     !get_pal_params_client_->wait_for_service(std::chrono::seconds(1)) ||
  //     !get_published_transforms_client_->wait_for_service(std::chrono::seconds(1)) ||
  //     !set_published_transforms_client_->wait_for_service(std::chrono::seconds(1)) ||
  //     !set_frame_client_->wait_for_service(std::chrono::seconds(1)) ||
  //     !set_tcp_client_->wait_for_service(std::chrono::seconds(1)) ||
  //     !set_pallet_side_client_->wait_for_service(std::chrono::seconds(1)))
  // {
  //   if (rclcpp::ok())
  //   {
  //     RCLCPP_ERROR(this->get_logger(),
  //                  "Client interrupted while waiting for service. Terminating...");
  //     rclcpp::shutdown();
  //     return;
  //   }
  //   RCLCPP_INFO(this->get_logger(),
  //               "Service Unavailable. Waiting for Service...");
  // }
  auto request = std::make_shared<uni_pal_msgs::srv::Empty::Request>();
  RCLCPP_INFO(this->get_logger(), "Sending request to create box on pallet");
  auto result_future = create_box_on_pp_client_->async_send_request(request);
  // result_future.wait();
  auto request2 = std::make_shared<uni_pal_msgs::srv::ExecuteTask::Request>();
  request2->task_nr = 0;
  RCLCPP_INFO(this->get_logger(), "Sending request to execute task 0");
  auto result_future2 = execute_task_client_->async_send_request(request2);
  // while (!result_future2.get()->success)
  // {
  //   RCLCPP_INFO(this->get_logger(), "Waiting for task execution...");
  //   std::this_thread::sleep_for(std::chrono::seconds(1));
  // }
  request2->task_nr = 100;
  RCLCPP_INFO(this->get_logger(), "Sending request to execute task 100");
  auto result_future3 = execute_task_client_->async_send_request(request2);
  // while (!result_future3.get()->success)
  // {
  //   RCLCPP_INFO(this->get_logger(), "Waiting for task execution...");
  //   std::this_thread::sleep_for(std::chrono::seconds(1));
  // }
  request2->task_nr = 200;
  RCLCPP_INFO(this->get_logger(), "Sending request to execute task 200");
  auto result_future4 = execute_task_client_->async_send_request(request2);
}

// Service client

