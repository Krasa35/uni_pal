#include "RobotClient/RobotClient.h"

RobotClient::RobotClient() : Node("robot_client"), pubished_msgs_count_(0)
{
  RCLCPP_INFO(this->get_logger(), "robot_client node has been started.");
  // Create publishers & publish
  dynamic_info_publisher_ = this->create_publisher<uni_pal_msgs::msg::RobotDynamicInfo>("/robot_client/dynamic_info", 10);
  static_info_publisher_ = this->create_publisher<uni_pal_msgs::msg::RobotStaticInfo>("/robot_client/static_info", 10);
  // Create subscribers
  robot_specific_subscriber_ = this->create_subscription<uni_pal_msgs::msg::RobotSpecific>(
      "/robot_specific", 10, std::bind(&RobotClient::robot_specific_subscriber_callback_, this, std::placeholders::_1));
  joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&RobotClient::joint_state_subscriber_callback_, this, std::placeholders::_1));
  tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  // Service Servers
  update_robot_static_info_srv_ = this->create_service<uni_pal_msgs::srv::Empty>(
      "/robot_client/update_robot_static_info", std::bind(&RobotClient::update_robot_static_info_, this, std::placeholders::_1, std::placeholders::_2));
  set_tcp_srv_ = this->create_service<uni_pal_msgs::srv::SetFrame>(
      "/robot_client/set_tcp", std::bind(&RobotClient::set_tcp_, this, std::placeholders::_1, std::placeholders::_2));
  set_frame_srv_ = this->create_service<uni_pal_msgs::srv::SetFrame>(
      "/robot_client/set_frame", std::bind(&RobotClient::set_frame_, this, std::placeholders::_1, std::placeholders::_2));
  // Service Clients
  config_params_client_ = this->create_client<uni_pal_msgs::srv::GetConfigParams>("/read_json_node/get_config_params");
  get_published_transforms_client_ = this->create_client<uni_pal_msgs::srv::GetPublishedTransforms>("/scene_client/get_published_transforms");
  got_config_params_ = false;
  got_robot_static_info_ = false;
  std::thread([this]()
              {
    rclcpp::WallRate loop_rate(std::chrono::milliseconds(100));
    while (rclcpp::ok()) {
      this->publish_messages_();
      loop_rate.sleep();
    } })
      .detach();
}

void RobotClient::check_topics_()
{
  rclcpp::Rate rate(0.1); // 1/10 Hz
  if (!got_config_params_) get_config_params_();
  while (robot_specific_subscriber_->get_publisher_count() == 0)
  {
    RCLCPP_ERROR(this->get_logger(), "No publishers for topic 'robot_specific'. Waiting...");
    rate.sleep();
  }
  while (joint_state_subscriber_->get_publisher_count() == 0)
  {
    RCLCPP_ERROR(this->get_logger(), "No publishers for topic 'joint_states'. Waiting...");
    rate.sleep();
  }
  while (!got_config_params_)
  {
    RCLCPP_ERROR(this->get_logger(), "No parameters from '/read_json_node/get_config_params' received. Waiting...");
    get_config_params_();
    rate.sleep();
  }
  while (!(tf_buffer->canTransform(static_message_.config.link_names.front(),
                                   static_message_.config.link_names.back(), 
                                   tf2::TimePointZero, tf2::durationFromSec(1.0))))
  {
    RCLCPP_ERROR(this->get_logger(), "No transforms available. Waiting...");
    rate.sleep();
  }
  while (!get_published_transforms_client_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(this->get_logger(), "Service '/scene_client/get_published_transforms' not available. Waiting...");
    rate.sleep();
  }
  if (!got_robot_static_info_) get_robot_static_info_();
  while (!got_robot_static_info_)
  {
    RCLCPP_ERROR(this->get_logger(), "No static info received. Waiting...");
    get_robot_static_info_();
    rate.sleep();
  }
}

// Publishers
void RobotClient::publish_messages_()
{
  check_topics_();
  get_robot_dynamic_info_();
  dynamic_info_publisher_->publish(dynamic_message_);
  dynamic_message_published_ = dynamic_message_;
  static_info_publisher_->publish(static_message_);
  static_message_published_ = static_message_;
  pubished_msgs_count_++;
}

// Subscribers
void RobotClient::robot_specific_subscriber_callback_(const uni_pal_msgs::msg::RobotSpecific &msg)
{
  dynamic_message_.analog = msg.analog;
  dynamic_message_.digital = msg.digital;
  return;
}

void RobotClient::joint_state_subscriber_callback_(const sensor_msgs::msg::JointState &msg)
{
  dynamic_message_.actual_pose.joint_state.resize(msg.position.size());
  dynamic_message_.actual_pose.joint_state = msg.position;
  return;
}

geometry_msgs::msg::TransformStamped RobotClient::lookup_transform_(const std::string &target_frame,
                                                                    const std::string &source_frame)
{
  geometry_msgs::msg::TransformStamped transform;
  try
  {
    transform = tf_buffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Transform error of %s to %s: %s",
                 target_frame.c_str(), source_frame.c_str(), ex.what());
  }
  return transform;
}
// Service Servers
void RobotClient::update_robot_static_info_(std::shared_ptr<uni_pal_msgs::srv::Empty::Request>,
                                            std::shared_ptr<uni_pal_msgs::srv::Empty::Response>)
{
  got_robot_static_info_ = false;
  got_config_params_ = false;
}
void RobotClient::set_frame_(std::shared_ptr<uni_pal_msgs::srv::SetFrame::Request> request,
                             std::shared_ptr<uni_pal_msgs::srv::SetFrame::Response> response)
{
  auto found = std::find(static_message_.predefined.frame_keys.begin(), 
               static_message_.predefined.frame_keys.end(), 
               request->frame);
  if (found != static_message_.predefined.frame_keys.end())
  {
    dynamic_message_.actual_pose.current_frame_name = request->frame;

    dynamic_message_.actual_pose.current_frame_pose.header.stamp = this->now();
    dynamic_message_.actual_pose.current_frame_pose.header.frame_id = "world";
    std::size_t index = std::distance(static_message_.predefined.frame_keys.begin(), found);
    dynamic_message_.actual_pose.current_frame_pose.pose = static_message_.predefined.frame_values[index];
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Current frame set to %s.", request->frame.c_str());
    return;
  }
  else
  {
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "Frame not found in predefined frames - check available in /robot_client/robot_static_info.");
    return;
  }
}
void RobotClient::set_tcp_(std::shared_ptr<uni_pal_msgs::srv::SetFrame::Request> request,
                           std::shared_ptr<uni_pal_msgs::srv::SetFrame::Response> response)
{
  auto found = std::find(static_message_.predefined.tcp_keys.begin(), 
               static_message_.predefined.tcp_keys.end(), 
               request->frame);
  if (found != static_message_.predefined.tcp_keys.end())
  {
    dynamic_message_.actual_pose.current_tcp_name = request->frame;

    dynamic_message_.actual_pose.current_tcp_pose.header.stamp = this->now();
    dynamic_message_.actual_pose.current_tcp_pose.header.frame_id = static_message_.config.link_names.back();
    std::size_t index = std::distance(static_message_.predefined.tcp_keys.begin(), found);
    dynamic_message_.actual_pose.current_tcp_pose.pose = static_message_.predefined.tcp_values[index];
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Current tcp set to %s.", request->frame.c_str());
    return;
  }
  else
  {
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "TCP not found in predefined TCPs - check available in /robot_client/robot_static_info.");
    return;
  }
}

// Service Clients
void RobotClient::get_config_params_()
{
  if (!config_params_client_->wait_for_service(std::chrono::seconds(5)))
  {
    if (rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Client interrupted while waiting for service. Terminating...");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "Service Unavailable. Waiting for Service...");
  }
  auto request = std::make_shared<uni_pal_msgs::srv::GetConfigParams_Request>();
  // set request variables here, if any
  got_config_params_ = false;
  auto result_future = config_params_client_->async_send_request(
      request, std::bind(&RobotClient::config_params_sent_service_, this,
                         std::placeholders::_1));
}

void RobotClient::get_published_transforms_()
{
  if (!get_published_transforms_client_->wait_for_service(std::chrono::seconds(1)))
  {
    if (rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Client interrupted while waiting for service. Terminating...");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "Service Unavailable. Waiting for Service...");
  }
  auto request = std::make_shared<uni_pal_msgs::srv::GetPublishedTransforms_Request>();
  // set request variables here, if any
  auto future = get_published_transforms_client_->async_send_request(request);
  try{
      auto response = future.get();
      for (auto &transform : response->transforms)
      {
        if (std::find(static_message_.predefined.frame_keys.begin(), static_message_.predefined.frame_keys.end(), transform.child_frame_id) != static_message_.predefined.frame_keys.end() ||
            std::find(static_message_.predefined.tcp_keys.begin(), static_message_.predefined.tcp_keys.end(), transform.child_frame_id) != static_message_.predefined.tcp_keys.end())
        {
          continue;
        }

        if (transform.header.frame_id == "world")
        {
          geometry_msgs::msg::Pose pose;
          pose.position.x = transform.transform.translation.x;
          pose.position.y = transform.transform.translation.y;
          pose.position.z = transform.transform.translation.z;
          pose.orientation = transform.transform.rotation;
          static_message_.predefined.frame_values.push_back(pose);
          static_message_.predefined.frame_keys.push_back(transform.child_frame_id);
        }
        else
        {
          geometry_msgs::msg::Pose pose;
          pose.position.x = transform.transform.translation.x;
          pose.position.y = transform.transform.translation.y;
          pose.position.z = transform.transform.translation.z;
          pose.orientation = transform.transform.rotation;
          static_message_.predefined.tcp_values.push_back(pose);
          static_message_.predefined.tcp_keys.push_back(transform.child_frame_id);
        }
      }
  }
  catch(const std::exception& e){
      RCLCPP_ERROR(this->get_logger(), "Service call failed");
  }
}

void RobotClient::config_params_sent_service_(rclcpp::Client<uni_pal_msgs::srv::GetConfigParams>::SharedFuture future)
{
  auto status = future.wait_for(std::chrono::seconds(1));
  if (status == std::future_status::ready)
  {
    uni_pal_msgs::srv::GetConfigParams_Response config_params_response = *(future.get());
    static_message_.config = config_params_response.config;
    got_config_params_ = true;
    RCLCPP_INFO(this->get_logger(), "Parameters Received from \"/read_json_node/get_config_params\".");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
  }
}

// Process methods
void RobotClient::get_robot_static_info_()
{
  static_message_.predefined.frame_keys = static_message_.config.frames;
  for (auto &frame : static_message_.predefined.frame_keys)
  {
    static_message_.predefined.frame_values.push_back(get_pose_(lookup_transform_("world", frame)));
  }
  static_message_.predefined.tcp_keys = static_message_.config.eefs;
  for (auto &tcp : static_message_.predefined.tcp_keys)
  {
    static_message_.predefined.tcp_values.push_back(get_pose_(lookup_transform_(static_message_.config.link_names.back(), tcp)));
  }
  get_published_transforms_();
  
  dynamic_message_.actual_pose.joint_state.resize(6);
  dynamic_message_.actual_pose.current_tcp_name = static_message_.config.link_names.back();
  dynamic_message_.actual_pose.current_frame_name = static_message_.config.link_names.front();
  got_robot_static_info_ = true;
  RCLCPP_INFO(this->get_logger(), "Static info received.");
}

void RobotClient::get_robot_dynamic_info_()
{
  geometry_msgs::msg::PoseStamped temp;
  temp.header.stamp = this->now();
  temp.header.frame_id = static_message_.config.link_names.back();
  temp.pose = get_pose_(lookup_transform_(static_message_.config.link_names.back(),
                                          dynamic_message_.actual_pose.current_tcp_name));
  dynamic_message_.actual_pose.current_tcp_pose = temp;

  temp.header.stamp = this->now();
  temp.header.frame_id = "world";
  temp.pose = get_pose_(lookup_transform_("world", dynamic_message_.actual_pose.current_frame_name));
  dynamic_message_.actual_pose.current_frame_pose = temp;

  geometry_msgs::msg::TransformStamped base_eef_transform = lookup_transform_(static_message_.config.link_names.front(), dynamic_message_.actual_pose.current_tcp_name);
  dynamic_message_.actual_pose.pose_in_base = get_pose_(base_eef_transform);
  dynamic_message_.actual_pose.distance_to_base = get_distance_(base_eef_transform);
  dynamic_message_.actual_pose.rpy_in_base = get_rpy_(base_eef_transform);

  geometry_msgs::msg::TransformStamped frame_eef_transform = lookup_transform_(dynamic_message_.actual_pose.current_frame_name, dynamic_message_.actual_pose.current_tcp_name);
  dynamic_message_.actual_pose.pose_in_base = get_pose_(frame_eef_transform);
  dynamic_message_.actual_pose.distance_to_base = get_distance_(frame_eef_transform);
  dynamic_message_.actual_pose.rpy_in_base = get_rpy_(frame_eef_transform);


  geometry_msgs::msg::TransformStamped world_eef_transform = lookup_transform_("world", dynamic_message_.actual_pose.current_tcp_name);
  dynamic_message_.actual_pose.pose_in_world = get_pose_(world_eef_transform);
  dynamic_message_.actual_pose.rpy_in_world = get_rpy_(world_eef_transform);

  process_robot_info_();
}

void RobotClient::process_robot_info_()
{
  geometry_msgs::msg::TransformStamped forearm_eef_transform = lookup_transform_("forearm_link", "flange");
  geometry_msgs::msg::Pose forearm_pose = get_pose_(forearm_eef_transform);
  geometry_msgs::msg::TransformStamped wrist1_eef_transform = lookup_transform_("wrist_1_link", "flange");
  geometry_msgs::msg::Pose wrist_1_pose = get_pose_(wrist1_eef_transform);

  double gripper_in_x = ((dynamic_message_.actual_pose.pose_in_base.position.x > 0.0) ? (-1) : (1)) *
                            (sin(abs(dynamic_message_.actual_pose.rpy_in_base.r_z)) * static_message_.config.gripper_length / 2 +
                             (abs(cos(dynamic_message_.actual_pose.rpy_in_base.r_z)) * static_message_.config.gripper_width / 2)) +
                        dynamic_message_.actual_pose.pose_in_base.position.x;
  double gripper_in_y = ((dynamic_message_.actual_pose.pose_in_base.position.y > 0.0) ? (-1) : (1)) *
                            (sin(abs(dynamic_message_.actual_pose.rpy_in_base.r_z)) * static_message_.config.gripper_width / 2 +
                             (abs(cos(dynamic_message_.actual_pose.rpy_in_base.r_z)) * static_message_.config.gripper_length / 2)) +
                        dynamic_message_.actual_pose.pose_in_base.position.y;
  double rad_j0 = dynamic_message_.actual_pose.joint_state[0] * M_PI / 180.0;

  dynamic_message_.state.over_pp = (dynamic_message_.actual_pose.pose_in_base.position.y > 0.8);
  dynamic_message_.state.high_above_base = (dynamic_message_.actual_pose.pose_in_base.position.z > 0.6);
  dynamic_message_.state.below_base = (dynamic_message_.actual_pose.pose_in_base.position.z < 0.0);
  dynamic_message_.state.over_base = (abs(gripper_in_x) < 0.2 && abs(gripper_in_y) < 0.9 && !dynamic_message_.state.over_pp);
  dynamic_message_.state.over_right_pallet = (dynamic_message_.actual_pose.pose_in_base.position.x > 0.2 && !dynamic_message_.state.over_base && !dynamic_message_.state.over_pp);
  dynamic_message_.state.over_left_pallet = (dynamic_message_.actual_pose.pose_in_base.position.x < -0.2 && !dynamic_message_.state.over_base && !dynamic_message_.state.over_pp);
  dynamic_message_.state.hyper_extension = (dynamic_message_.actual_pose.distance_to_base > static_message_.config.max_reach - 0.1);
  dynamic_message_.state.hyper_flexion = (dynamic_message_.actual_pose.distance_to_base < 0.6);
  dynamic_message_.state.hyper_roll = (dynamic_message_.actual_pose.joint_state[0] < -90 || dynamic_message_.actual_pose.joint_state[0] > 270 ||
                                       (dynamic_message_.actual_pose.joint_state[0] < 0 && dynamic_message_.actual_pose.pose_in_base.position.x < 0.0) ||
                                       (dynamic_message_.actual_pose.joint_state[0] > 180 && dynamic_message_.actual_pose.pose_in_base.position.x > 0.0));
  dynamic_message_.state.forearm_gripper_warn = (dynamic_message_.actual_pose.pose_in_base.position.z + static_message_.config.gripper_offset > ((wrist_1_pose.position.z - forearm_pose.position.z) /
                                                                                                                                                 ((wrist_1_pose.position.y * sin(rad_j0) + wrist_1_pose.position.x * cos(rad_j0)) - (forearm_pose.position.y * sin(rad_j0) + forearm_pose.position.x * cos(rad_j0)))) *
                                                                                                                                                        ((gripper_in_y * sin(rad_j0) + gripper_in_x * cos(rad_j0)) - (forearm_pose.position.y * sin(rad_j0) + forearm_pose.position.x * cos(rad_j0))) +
                                                                                                                                                    forearm_pose.position.z);
  dynamic_message_.state.product_on_gripper = 0;
}

geometry_msgs::msg::Pose RobotClient::get_pose_(const geometry_msgs::msg::TransformStamped &transform_stamped)
{
  geometry_msgs::msg::Pose result;
  tf2::Transform transform_;
  tf2::fromMsg(transform_stamped.transform, transform_);

  tf2::Quaternion q_frame = transform_.getRotation();
  tf2::Vector3 t_frame = transform_.getOrigin();

  result.orientation.x = q_frame.x();
  result.orientation.y = q_frame.y();
  result.orientation.z = q_frame.z();
  result.orientation.w = q_frame.w();

  result.position.x = t_frame.x();
  result.position.y = t_frame.y();
  result.position.z = t_frame.z();

  return result;
}

double RobotClient::get_distance_(const geometry_msgs::msg::TransformStamped &transform_stamped)
{
  double result;
  tf2::Transform transform_;
  tf2::fromMsg(transform_stamped.transform, transform_);
  tf2::Vector3 t_frame = transform_.getOrigin();
  result = sqrt(pow(t_frame.x(), 2) + pow(t_frame.y(), 2) + pow(t_frame.z(), 2));
  return result;
}

uni_pal_msgs::msg::RobotRPY RobotClient::get_rpy_(const geometry_msgs::msg::TransformStamped &transform_stamped)
{
  uni_pal_msgs::msg::RobotRPY result;
  tf2::Transform transform_;
  tf2::fromMsg(transform_stamped.transform, transform_);
  tf2::Quaternion q = transform_.getRotation();
  tf2::Matrix3x3 m(q);
  m.getRPY(result.r_x, result.r_y, result.r_z);
  result.r_x = result.r_x * 180 / M_PI;
  result.r_y = result.r_y * 180 / M_PI;
  result.r_z = result.r_z * 180 / M_PI;
  return result;
}
