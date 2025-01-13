#include "SceneClient/SceneClient.h"

SceneClient::SceneClient() : Node("scene_client")
{
  RCLCPP_INFO(this->get_logger(), "scene_client node has been started.");
  // Publishers
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  pal_params_publisher_ = this->create_publisher<uni_pal_msgs::msg::PalParams>("/scene_client/pal_params", 10);
  counters_publisher_ = this->create_publisher<uni_pal_msgs::msg::Counters>("/scene_client/counters", 10);
  planning_scene_diff_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
  
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
  planning_scene_diff_client_ = this->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
  got_pal_params_ = false;
  // Other
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_ros::CreateTimerInterface::SharedPtr cti = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(cti);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  frame_points_.resize(3);
  std::thread([this]() {
    rclcpp::WallRate loop_rate(std::chrono::milliseconds(100));
    while (rclcpp::ok()) {
      this->publish_messages_();
      loop_rate.sleep();
    }
  }).detach();
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
      RCLCPP_INFO(this->get_logger(), "Publishing transforms...");
      transform.header.stamp = this->now();
      tf_broadcaster_->sendTransform(transform);
    }
  }

  counters_publisher_->publish(counters_msg_);
  auto timeout_ms_ = std::chrono::milliseconds(100);

  if(collision_objects_.size() > 0)
  {
    while (!transform_available_ && rclcpp::ok()) {
      static constexpr auto input = "world", output = "pickpoint";
      RCLCPP_INFO(this->get_logger(), "waiting %ld ms for %s->%s transform to become available",
                  timeout_ms_.count(), input, output);
      auto callback = [this](const std::shared_future<geometry_msgs::msg::TransformStamped> & future) {
        try {
          auto tf = future.get();
          transform_available_ = true;
        } catch (const tf2::TimeoutException & e) {
          RCLCPP_INFO(get_logger(), "hit TimeoutException: %s", e.what());
        }
      };
      auto future = tf_buffer_->waitForTransform(
                                  input, output, tf2::TimePointZero, std::chrono::milliseconds(timeout_ms_), callback);
      future.wait_for(timeout_ms_);
    }
    RCLCPP_INFO(this->get_logger(), "Publishing collision objects...");
    for (auto &object : collision_objects_)
    {
      planning_scene_.world.collision_objects.push_back(object.object);
    }
    planning_scene_.is_diff = true;
    planning_scene_diff_publisher_->publish(planning_scene_);
    // transform_available_ = false;
  }
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
    RCLCPP_INFO(this->get_logger(), "Waiting for parameters...");
    rate.sleep();
  }

  // Wait for the "pickpoint" transform to be available
  while (!tf_buffer_->canTransform("world", "pickpoint", tf2::TimePointZero) && rclcpp::ok())
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for 'pickpoint' transform to become available...");
    rate.sleep();
  }
  geometry_msgs::msg::TransformStamped pp_to_world = tf_buffer_->lookupTransform("world", "pickpoint", tf2::TimePointZero);

  moveit_msgs::msg::AttachedCollisionObject box;
  box.link_name = "tool0";
  box.object.id = "box " + std::to_string(counters_msg_.total_boxes_placed);
  box.object.header.frame_id = "world";
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = pal_params_response_.params.box.length / 1000;
  primitive.dimensions[1] = pal_params_response_.params.box.width / 1000;
  primitive.dimensions[2] = pal_params_response_.params.box.height / 1000;

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = pp_to_world.transform.translation.x - pal_params_response_.params.box.length / 2000;
  pose.position.y = pp_to_world.transform.translation.y + pal_params_response_.params.box.width / 2000;
  pose.position.z = pp_to_world.transform.translation.z + pal_params_response_.params.box.height / 2000;

  box.object.primitives.push_back(primitive);
  box.object.primitive_poses.push_back(pose);
  box.object.operation = box.object.ADD;
  box.touch_links = std::vector<std::string>{ "gripper" };

  collision_objects_.push_back(box);
  RCLCPP_INFO(this->get_logger(), "Box added to pickpoint");
  increment_counters_();
  // send_tf_request_();
}

void SceneClient::set_pallet_side_(std::shared_ptr<uni_pal_msgs::srv::SetPalletSide::Request> request,
                                    std::shared_ptr<uni_pal_msgs::srv::SetPalletSide::Response>)
{
  std::regex set_pattern(R"((set) layer=(\d+) box=(\d+))", std::regex_constants::icase);
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
    cnt->boxes_placed = cnt->boxes_on_current_layer + (cnt->layers_placed) * cnt->boxes_per_layer;
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

// void SceneClient::send_tf_request_()
// {
//   planning_scene_diff_client_->wait_for_service();
//   auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
//   moveit_msgs::msg::PlanningScene planning_scene;
//   planning_scene_.world.collision_objects.push_back(collision_object.object);
//   planning_scene_.is_diff = true;
//   request->scene = planning_scene;
//   // std::shared_future<std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene_Response>> response_future;
//   // response_future = planning_scene_diff_client_->async_send_request(request).future.share();
//     // auto response_future = std::async(std::launch::async, [this, request]() {
//     //   return planning_scene_diff_client_->async_send_request(request).get();
//     // });
//   auto result_future = planning_scene_diff_client_->async_send_request(request).future.share();
//   // wait for the service to respond
//   std::chrono::seconds wait_time(100);
//   std::future_status fs = result_future.wait_for(wait_time);
//   if (fs == std::future_status::ready)
//   {
//     std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene_Response> planning_response;
//     planning_response = result_future.get();
//     if (planning_response->success)
//     {
//       RCLCPP_INFO(this->get_logger(), "Service successfully added object.");
//     }
//     else
//     {
//       RCLCPP_ERROR(this->get_logger(), "Service failed to add object.");
//     }
//   }
//   else
//   {
//     RCLCPP_ERROR(this->get_logger(), "Service timed out.");
//   }
// }

// OTHER
void SceneClient::increment_counters_()
{
  uni_pal_msgs::msg::PalletCounters *cnt;
  cnt = &counters_msg_.left_pallet;
  if (cnt->pallet_state == "FULL") return;
  counters_msg_.total_boxes_placed++;
  cnt->layers_placed = floor(counters_msg_.total_boxes_placed % pal_params_response_.params.boxes_per_layer);
  cnt->boxes_on_current_layer = counters_msg_.total_boxes_placed % pal_params_response_.params.boxes_per_layer;
  if (cnt->layers_placed == cnt->layers_per_pallet) cnt->pallet_state = "FULL";
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
  t = computeConveyorFrame();
  published_transforms_.push_back(t);
}

geometry_msgs::msg::TransformStamped SceneClient::computeConveyorFrame(){
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