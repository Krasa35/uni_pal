#include "RobotClient/RobotClient.h"

RobotClient::RobotClient() : Node("robot_client"), pubished_msgs_count_(0)
{
  RCLCPP_INFO(this->get_logger(), "robot_client node has been started.");
  // Create publishers & publish
  dynamic_info_publisher_ = this->create_publisher<uni_pal_msgs::msg::RobotDynamicInfo>("/robot_client/dynamic_info", 10);
  static_info_publisher_ = this->create_publisher<uni_pal_msgs::msg::RobotStaticInfo>("/robot_client/static_info", 10);
  publish_messages_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&RobotClient::publish_messages_, this));
  // Create subscribers
  robot_specific_subscriber_ = this->create_subscription<uni_pal_msgs::msg::RobotSpecific>(
    "/robot_specific", 10, std::bind(&RobotClient::robot_specific_subscriber_callback_, this, std::placeholders::_1));
  joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, std::bind(&RobotClient::joint_state_subscriber_callback_, this, std::placeholders::_1));
  tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  // Service Clients
  config_params_client_ = this->create_client<uni_pal_msgs::srv::GetConfigParams>("/read_json_node/get_config_params");
  got_config_params_ = false;
  get_robot_static_info_();
}

void RobotClient::check_topics_()
{
    if (robot_specific_subscriber_->get_publisher_count() == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "No publishers for topic 'robot_specific'. Terminating...");
        rclcpp::shutdown();
    }
    if (joint_state_subscriber_->get_publisher_count() == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "No publishers for topic 'joint_states'. Terminating...");
        rclcpp::shutdown();
    }
    if (!(tf_buffer->canTransform("base_link", "flange", tf2::TimePointZero, tf2::durationFromSec(1.0))))
    {
        RCLCPP_ERROR(this->get_logger(), "No transforms available. Terminating...");
        rclcpp::shutdown();
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
void RobotClient::robot_specific_subscriber_callback_(const uni_pal_msgs::msg::RobotSpecific& msg)
{
  dynamic_message_.analog = msg.analog;
  dynamic_message_.digital = msg.digital;
  return;
}

void RobotClient::joint_state_subscriber_callback_(const sensor_msgs::msg::JointState& msg)
{
  dynamic_message_.actual_pose.joint_state.resize(msg.position.size());
  dynamic_message_.actual_pose.joint_state = msg.position;
  return;
}

geometry_msgs::msg::TransformStamped RobotClient::lookup_transform_(const std::string& target_frame, 
                                                                    const std::string& source_frame)
{
  geometry_msgs::msg::TransformStamped transform;
  try
  {
    transform = tf_buffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Transform error of %s to %s: %s",
                 target_frame.c_str(), source_frame.c_str(), ex.what());
  }
  return transform;
}

// Service Clients
void RobotClient::get_config_params_()
{
    if (!config_params_client_->wait_for_service(std::chrono::seconds(1))) 
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
  get_config_params_();
  dynamic_message_.actual_pose.joint_state.resize(6);
}

void RobotClient::get_robot_dynamic_info_()
{
  geometry_msgs::msg::TransformStamped base_eef_transform = lookup_transform_("base_link", "flange");
  dynamic_message_.actual_pose.pose_in_base = get_pose_(base_eef_transform);
  dynamic_message_.actual_pose.distance_to_base = get_distance_(base_eef_transform);
  dynamic_message_.actual_pose.rpy_in_base = get_rpy_(base_eef_transform);

  geometry_msgs::msg::TransformStamped world_eef_transform = lookup_transform_("world", "flange");
  dynamic_message_.actual_pose.pose_in_world = get_pose_(world_eef_transform);
  dynamic_message_.actual_pose.rpy_in_world = get_rpy_(world_eef_transform);

  dynamic_message_.actual_pose.current_tcp_name = "tool0";
  geometry_msgs::msg::PoseStamped temp;
  temp.header.stamp = this->now();
  temp.header.frame_id = "tool0";
  temp.pose = get_pose_(lookup_transform_("tool0", "tool0"));
  dynamic_message_.actual_pose.current_tcp_pose = temp;

  process_robot_info_();
}

void RobotClient::process_robot_info_()
{
  geometry_msgs::msg::TransformStamped forearm_eef_transform = lookup_transform_("forearm_link", "flange");
  geometry_msgs::msg::Pose forearm_pose = get_pose_(forearm_eef_transform);
  geometry_msgs::msg::TransformStamped wrist1_eef_transform = lookup_transform_("wrist_1_link", "flange");
  geometry_msgs::msg::Pose wrist_1_pose = get_pose_(wrist1_eef_transform);

  double gripper_in_x = ((dynamic_message_.actual_pose.pose_in_base.position.x > 0.0) ? (-1) : (1))*
                      (sin(abs(dynamic_message_.actual_pose.rpy_in_base.r_z))*static_message_.config.gripper_length/2 +
                      (abs(cos(dynamic_message_.actual_pose.rpy_in_base.r_z))*static_message_.config.gripper_width/2)) +
                      dynamic_message_.actual_pose.pose_in_base.position.x;
  double gripper_in_y = ((dynamic_message_.actual_pose.pose_in_base.position.y > 0.0) ? (-1) : (1))*
                      (sin(abs(dynamic_message_.actual_pose.rpy_in_base.r_z))*static_message_.config.gripper_width/2 +
                      (abs(cos(dynamic_message_.actual_pose.rpy_in_base.r_z))*static_message_.config.gripper_length/2)) +
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
                                      ((wrist_1_pose.position.y * sin(rad_j0) + wrist_1_pose.position.x * cos(rad_j0)) - (forearm_pose.position.y * sin(rad_j0) + forearm_pose.position.x * cos(rad_j0))))*
                                      ((gripper_in_y * sin(rad_j0) + gripper_in_x * cos(rad_j0)) - (forearm_pose.position.y * sin(rad_j0) + forearm_pose.position.x * cos(rad_j0))) + forearm_pose.position.z);
  dynamic_message_.state.product_on_gripper = 0;
}

geometry_msgs::msg::Pose RobotClient::get_pose_(const geometry_msgs::msg::TransformStamped& transform_stamped)
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

double RobotClient::get_distance_(const geometry_msgs::msg::TransformStamped& transform_stamped){
  double result;
  tf2::Transform transform_;
  tf2::fromMsg(transform_stamped.transform, transform_);
  tf2::Vector3 t_frame = transform_.getOrigin();
  result = sqrt(pow(t_frame.x(), 2) + pow(t_frame.y(), 2) + pow(t_frame.z(), 2));
  return result;
}

uni_pal_msgs::msg::RobotRPY RobotClient::get_rpy_(const geometry_msgs::msg::TransformStamped& transform_stamped)
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

