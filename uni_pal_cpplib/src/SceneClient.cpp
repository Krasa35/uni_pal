#include "SceneClient/SceneClient.h"

SceneClient::SceneClient() : Node("scene_client")
{
  RCLCPP_INFO(this->get_logger(), "scene_client node has been started.");
  // Publishers
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  pal_params_publisher_ = this->create_publisher<uni_pal_msgs::msg::PalParams>("/scene_client/pal_params", 10);
  counters_publisher_ = this->create_publisher<uni_pal_msgs::msg::Counters>("/scene_client/counters", 10);
  planning_scene_diff_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
  // Subscribers
  static_message_subscriber_ = this->create_subscription<uni_pal_msgs::msg::RobotStaticInfo>(
      "/robot_client/static_info", 10, std::bind(&SceneClient::static_message_subscriber_callback_, this, std::placeholders::_1));
  // Service Servers
  get_pal_params_srv_ = this->create_service<uni_pal_msgs::srv::Empty>(
      "/scene_client/get_pal_params", std::bind(&SceneClient::get_pal_params_, this, std::placeholders::_1, std::placeholders::_2));
  get_published_transforms_srv = this->create_service<uni_pal_msgs::srv::GetPublishedTransforms>(
      "/scene_client/get_published_transforms", std::bind(&SceneClient::get_published_transforms_, this, std::placeholders::_1, std::placeholders::_2));
  set_published_transforms_srv = this->create_service<uni_pal_msgs::srv::SetPublishedTransforms>(
      "/scene_client/set_published_transforms", std::bind(&SceneClient::set_published_transforms_, this, std::placeholders::_1, std::placeholders::_2));
  create_box_on_pp_srv = this->create_service<uni_pal_msgs::srv::Empty>(
      "/scene_client/create_box_on_pp", std::bind(&SceneClient::create_box_on_pp_, this, std::placeholders::_1, std::placeholders::_2));
  set_pallet_side_srv_ = this->create_service<uni_pal_msgs::srv::SetPalletSide>(
      "/scene_client/set_pallet_side", std::bind(&SceneClient::set_pallet_side_, this, std::placeholders::_1, std::placeholders::_2));
  // Service Clients
  pal_params_client_ = this->create_client<uni_pal_msgs::srv::GetPalParams>("/read_json_node/get_pallet_params");
  got_pal_params_ = false;
  // Other
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_ros::CreateTimerInterface::SharedPtr cti = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(cti);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  frame_points_.resize(3);
  std::thread([this]()
              {
    rclcpp::WallRate loop_rate(std::chrono::milliseconds(100));
    while (rclcpp::ok()) {
      this->publish_messages_();
      loop_rate.sleep();
    } })
      .detach();
}
// Publishers
void SceneClient::publish_messages_()
{
  if (got_pal_params_)
  {
    uni_pal_msgs::msg::PalParams msg = pal_params_response_.params;
    pal_params_publisher_->publish(msg);
  }

  if (published_transforms_.size() > 0)
  {
    for (auto &transform : published_transforms_)
    {
      transform.header.stamp = this->now();
      tf_broadcaster_->sendTransform(transform);
    }
  }

  counters_publisher_->publish(counters_msg_);

}
// Subscribers
void SceneClient::static_message_subscriber_callback_(const uni_pal_msgs::msg::RobotStaticInfo& msg)
{
  static_message_ = msg;
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

void SceneClient::create_box_on_pp_(std::shared_ptr<uni_pal_msgs::srv::Empty::Request>,
                                    std::shared_ptr<uni_pal_msgs::srv::Empty::Response>)
{
  init_pickpoint_();
  rclcpp::Rate rate(0.1); // 1/10 Hz
  while (!got_pal_params_)
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for parameters... Call /scene_client/get_pal_params service first.");
    rate.sleep();
  }

  // Wait for the "pickpoint" transform to be available
  while (!tf_buffer_->canTransform("world", "pickpoint", tf2::TimePointZero) && rclcpp::ok())
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for 'pickpoint' transform to become available...");
    rate.sleep();
  }
  geometry_msgs::msg::TransformStamped pp_to_world = tf_buffer_->lookupTransform("world", "pickpoint", tf2::TimePointZero);

  float box_x = (pal_params_response_.params.programsettings.box_orientation == "SHORT_SIDE_LEADING") ? pal_params_response_.params.box.length : pal_params_response_.params.box.width;
  float box_y = (pal_params_response_.params.programsettings.box_orientation == "SHORT_SIDE_LEADING") ? pal_params_response_.params.box.width : pal_params_response_.params.box.length;
  float box_z = pal_params_response_.params.box.height;
  box_x = box_x/1000;
  box_y = box_y/1000;
  box_z = box_z/1000;

  moveit_msgs::msg::AttachedCollisionObject box;
  // box.link_name = "tool0";
  box.object.id = "box " + std::to_string(counters_msg_.total_boxes_placed+1);
  box.object.header.frame_id = "world";
  box.object.primitives.resize(2);
  shape_msgs::msg::SolidPrimitive primitive;
  box.object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  box.object.primitives[0].dimensions = {box_x, box_y, box_z};
  box.object.primitives[1].type = shape_msgs::msg::SolidPrimitive::BOX;
  box.object.primitives[1].dimensions = {box_x / 2, box_y / 2, box_z / 2};
  box.object.primitive_poses.resize(2);
  box.object.pose.position.x = pp_to_world.transform.translation.x - box_y / 2; //because in world frame, box is rotated
  box.object.pose.position.y = pp_to_world.transform.translation.y + box_x / 2; //because in world frame, box is rotated
  box.object.pose.position.z = pp_to_world.transform.translation.z + box_z / 2;
  box.object.pose.orientation = pp_to_world.transform.rotation;
  if (pal_params_response_.params.programsettings.label_position != "BACK" &&
      pal_params_response_.params.programsettings.label_position != "FRONT")
  {
    if (pal_params_response_.params.programsettings.label_position == "LEFT" ||
        pal_params_response_.params.programsettings.label_position == "LEFT_BACK_CORNER" ||
        pal_params_response_.params.programsettings.label_position == "LEFT_FRONT_CORNER")
    {
      box.object.primitive_poses[1].position.y = box_y / 3.9 * (-1);
    }
    else
    {
      box.object.primitive_poses[1].position.y = box_y / 3.9;
    }
  }
  if (pal_params_response_.params.programsettings.label_position != "LEFT" &&
      pal_params_response_.params.programsettings.label_position != "RIGHT")
  {
    if (pal_params_response_.params.programsettings.label_position == "FRONT" ||
        pal_params_response_.params.programsettings.label_position == "LEFT_FRONT_CORNER" ||
        pal_params_response_.params.programsettings.label_position == "RIGHT_FRONT_CORNER")
    {
      box.object.primitive_poses[1].position.x = box_x / 3.9 * (-1);
    }
    else
    {
      box.object.primitive_poses[1].position.x = box_x / 3.9;
    }
  }

  box.object.operation = box.object.ADD;
  box.touch_links = std::vector<std::string>{"gripper"};

  collision_objects_.push_back(box);
  planning_scene_.world.collision_objects.push_back(box.object);
  planning_scene_.is_diff = true;
  planning_scene_diff_publisher_->publish(planning_scene_);
  RCLCPP_INFO(this->get_logger(), "Box added to pickpoint");
  increment_counters_();
}

void SceneClient::set_pallet_side_(std::shared_ptr<uni_pal_msgs::srv::SetPalletSide::Request> request,
                                   std::shared_ptr<uni_pal_msgs::srv::SetPalletSide::Response>)
{
  std::regex set_pattern(R"((set)_layer_(\d+)_box_(\d+))", std::regex_constants::icase);
  std::smatch match;
  uni_pal_msgs::msg::PalletCounters *cnt;

  if (request->side != "left" && request->side != "right")
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid side. Must be either 'left' or 'right'");
    return;
  }
  cnt = (request->side == "right") ? &counters_msg_.right_pallet : &counters_msg_.left_pallet;

  if (request->action == "reset")
  {
    cnt->pallet_state = "EMPTY";
    cnt->layers_placed = 0;
    cnt->boxes_on_current_layer = 0;
    cnt->boxes_placed = 0;
  }
  else if (std::regex_search(request->action, match, set_pattern))
  {
    cnt->pallet_state = "IN_PROGRESS";
    cnt->layers_placed = std::stoi(match[2]);
    cnt->boxes_on_current_layer = std::stoi(match[3]);
    cnt->boxes_placed = cnt->boxes_on_current_layer + (cnt->layers_placed) * pal_params_response_.params.boxes_per_layer;
  }
  else  
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid action. Must be either 'reset' or 'set_layer_<layer_nr>_box_<box_nr>'");
    return;
  }
}

// Service Clients
void SceneClient::pal_params_sent_service_(rclcpp::Client<uni_pal_msgs::srv::GetPalParams>::SharedFuture future)
{
  auto status = future.wait_for(std::chrono::seconds(1));
  if (status == std::future_status::ready)
  {
    pal_params_response_ = *(future.get());
    got_pal_params_ = true;
  }
}

// OTHER
void SceneClient::increment_counters_()
{
  uni_pal_msgs::msg::PalletCounters *cnt;
  cnt = &counters_msg_.left_pallet;
  if (cnt->pallet_state == "FULL")
    return;
  counters_msg_.total_boxes_placed++;
  cnt->layers_placed = floor(counters_msg_.total_boxes_placed / pal_params_response_.params.boxes_per_layer);
  cnt->boxes_on_current_layer = counters_msg_.total_boxes_placed % pal_params_response_.params.boxes_per_layer;
  if (cnt->layers_placed == pal_params_response_.params.layers_per_pallet)
    cnt->pallet_state = "FULL";
  cnt->pallet_state = "IN_PROGRESS";
}

void SceneClient::init_pickpoint_()
{
  geometry_msgs::msg::TransformStamped t;
  geometry_msgs::msg::PoseStamped temp;

  temp.header.frame_id = "world";
  temp.pose.position.x = 0.220748;
  temp.pose.position.y = 0.772368;
  temp.pose.position.z = 0.868602;
  frame_points_[0] = geometry_msgs::msg::PoseStamped(temp);
  temp.pose.position.x = 0.220748;
  temp.pose.position.y = 1.076969;
  temp.pose.position.z = 0.868602;
  frame_points_[1] = geometry_msgs::msg::PoseStamped(temp);
  temp.pose.position.x = -0.157520;
  temp.pose.position.y = 0.772970;
  temp.pose.position.z = 0.868602;
  frame_points_[2] = geometry_msgs::msg::PoseStamped(temp);
  published_transforms_.push_back(computeConveyorFrame());

  t.header.frame_id = static_message_.config.link_names.back();
  RCLCPP_INFO(this->get_logger(), "Pickpoint frame: %s", t.header.frame_id.c_str());
  t.child_frame_id = "gripper_pads";
  auto found = std::find(static_message_.predefined.tcp_keys.begin(), static_message_.predefined.tcp_keys.end(), "gripper");
  std::size_t index = std::distance(static_message_.predefined.tcp_keys.begin(), found);
  RCLCPP_INFO(this->get_logger(), "Gripper index: %ld", index);
  t.transform.translation.z = 0.1;
  RCLCPP_INFO(this->get_logger(), "Gripper pads frame: %s", t.child_frame_id.c_str());
  published_transforms_.push_back(t);
}

geometry_msgs::msg::TransformStamped SceneClient::computeConveyorFrame()
{
  geometry_msgs::msg::TransformStamped conveyor_frame_;

  tf2::Vector3 point_x(frame_points_[1].pose.position.x - frame_points_[0].pose.position.x, frame_points_[1].pose.position.y - frame_points_[0].pose.position.y, frame_points_[1].pose.position.z - frame_points_[0].pose.position.z);
  tf2::Vector3 point_y(frame_points_[2].pose.position.x - frame_points_[0].pose.position.x, frame_points_[2].pose.position.y - frame_points_[0].pose.position.y, frame_points_[2].pose.position.z - frame_points_[0].pose.position.z);

  point_x = point_x.normalize();
  point_y = point_y.normalize();

  tf2::Vector3 point_z = point_x.cross(point_y);

  tf2::Matrix3x3 rotation_matrix(point_x.x(), point_y.x(), point_z.x(),
                                 point_x.y(), point_y.y(), point_z.y(),
                                 point_x.z(), point_y.z(), point_z.z());
  tf2::Quaternion q;
  rotation_matrix.getRotation(q);
  conveyor_frame_.header.stamp = this->now();
  conveyor_frame_.header.frame_id = "world";
  conveyor_frame_.child_frame_id = "pickpoint";
  conveyor_frame_.transform.translation.x = frame_points_[0].pose.position.x;
  conveyor_frame_.transform.translation.y = frame_points_[0].pose.position.y;
  conveyor_frame_.transform.translation.z = frame_points_[0].pose.position.z;
  conveyor_frame_.transform.rotation = tf2::toMsg(q);
  return conveyor_frame_;
}