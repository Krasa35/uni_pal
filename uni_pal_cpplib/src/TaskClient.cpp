#include "TaskClient/TaskClient.h"

TaskClient::TaskClient(const rclcpp::NodeOptions &options) : Node("task_client", options)
{
  // Subscribers
  static_info_subscriber_ = this->create_subscription<uni_pal_msgs::msg::RobotStaticInfo>(
      "/robot_client/static_info", 10, std::bind(&TaskClient::static_info_subscriber_callback_, this, std::placeholders::_1));
  dynamic_info_subscriber_ = this->create_subscription<uni_pal_msgs::msg::RobotDynamicInfo>(
      "/robot_client/dynamic_info", 10, std::bind(&TaskClient::dynamic_info_subscriber_callback_, this, std::placeholders::_1));
  pallet_params_subscriber_ = this->create_subscription<uni_pal_msgs::msg::PalParams>(
      "/scene_client/pal_params", 10, std::bind(&TaskClient::pallet_params_subscriber_callback_, this, std::placeholders::_1));
  counters_subscriber_ = this->create_subscription<uni_pal_msgs::msg::Counters>(
      "/scene_client/counters", 10, std::bind(&TaskClient::counters_subscriber_callback_, this, std::placeholders::_1));
  // Service servers
  execute_task_srv_ = this->create_service<uni_pal_msgs::srv::ExecuteTask>(
      "/task_client/execute_task", std::bind(&TaskClient::execute_task_, this, std::placeholders::_1, std::placeholders::_2));
  // Service clients
  get_place_pose_client_ = this->create_client<uni_pal_msgs::srv::GetPlacePos>("/read_json_node/get_box_position");
}

// Subscribers
void TaskClient::static_info_subscriber_callback_(const uni_pal_msgs::msg::RobotStaticInfo &msg)
{
  static_message_ = msg;
}

void TaskClient::dynamic_info_subscriber_callback_(const uni_pal_msgs::msg::RobotDynamicInfo &msg)
{
  dynamic_message_ = msg;
}

void TaskClient::pallet_params_subscriber_callback_(const uni_pal_msgs::msg::PalParams &msg)
{
  pallet_params_message_ = msg;
}

void TaskClient::counters_subscriber_callback_(const uni_pal_msgs::msg::Counters &msg)
{
  counters_message_ = msg;
}

// Service server
void TaskClient::execute_task_(std::shared_ptr<uni_pal_msgs::srv::ExecuteTask::Request> request,
                               std::shared_ptr<uni_pal_msgs::srv::ExecuteTask::Response> response)
{
  RobotMovement move = toRobotMovement(request->task_nr);
  response->success = do_task_(move);
}

// Service Clients
void TaskClient::get_place_pose_(uni_pal_msgs::srv::GetPlacePos::Request request)
{
  RCLCPP_INFO(this->get_logger(), "Waiting for service '/read_json_node/get_box_position'");
  if (!get_place_pose_client_->wait_for_service(std::chrono::seconds(1)))
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
  RCLCPP_INFO(this->get_logger(), "Service available. Sending request...");
  std::shared_ptr<uni_pal_msgs::srv::GetPlacePos::Request> request_to_send = std::make_shared<uni_pal_msgs::srv::GetPlacePos::Request>();
  request_to_send->layer_no = request.layer_no;
  request_to_send->box_no = request.box_no;
  request_to_send->pallet_side = request.pallet_side.c_str();
  RCLCPP_INFO(this->get_logger(), "Requesting place pose for layer %d, box %d on %s pallet", request_to_send->layer_no, request_to_send->box_no, request_to_send->pallet_side.c_str());
  auto result_future = get_place_pose_client_->async_send_request(
      request_to_send, std::bind(&TaskClient::get_place_pose_sent_service, this,
                         std::placeholders::_1));
}

void TaskClient::get_place_pose_sent_service(rclcpp::Client<uni_pal_msgs::srv::GetPlacePos>::SharedFuture future)
{
  auto status = future.wait_for(std::chrono::seconds(1));
  if (status == std::future_status::ready)
  {
    uni_pal_msgs::srv::GetPlacePos_Response_ response = *(future.get());
    place_pose_.place_pose.position.x = response.place_pose.position.x / 1000;
    place_pose_.place_pose.position.y = response.place_pose.position.y / 1000;
    place_pose_.place_pose.position.z = (counters_message_.left_pallet.layers_placed + 1) * pallet_params_message_.box.height / 1000 + pallet_params_message_.pallet.height / 1000;
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, place_pose_.place_pose.position.z);
    place_pose_.place_pose.orientation = tf2::toMsg(q);
    RCLCPP_INFO(this->get_logger(), "Parameters Received from \"/read_json_node/get_box_position\".");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
  }
}

// MoveIt Task Constructor
bool TaskClient::do_task_(RobotMovement task_nr)
{
  if (static_message_.config.arm_group_name.empty() || dynamic_message_.actual_pose.current_tcp_name.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Robot info not available");
    return false;
  }
  uni_pal_msgs::srv::GetPlacePos::Request request;
  switch (task_nr)
  {
  case RobotMovement::Homing: // 0
    task_ = create_homing_task_();
    break;
  case RobotMovement::EmergencyHoming: // 1
    task_ = create_emergency_homing_task_();
    break;
  case RobotMovement::Pick: // 100
    task_ = create_pick_task_();
    break;
  case RobotMovement::Place: // 200
    request.layer_no = counters_message_.left_pallet.layers_placed + 1;
    request.box_no = counters_message_.left_pallet.boxes_on_current_layer; // no +1 because it is in array
    request.pallet_side = "LEFT";
    RCLCPP_INFO(this->get_logger(), "Requesting place pose for layer %d, box %d on %s pallet", request.layer_no, request.box_no, request.pallet_side.c_str());
    get_place_pose_(request);
    task_ = create_place_task_();
    break;
  case RobotMovement::Demo: // 999
    task_ = create_demo_task_();
    break;
  default:
    throw std::invalid_argument("Invalid task_nr value");
    break;
  }

  try
  {
    task_.init();
  }
  catch (moveit::task_constructor::InitStageException &e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), e);
    return false;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Task planning failed");
    return false;
  }
  task_.introspection().publishSolution(*task_.solutions().front());
  auto action_client = rclcpp_action::create_client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>(shared_from_this(), "execute_task_solution");

  if (!action_client->wait_for_action_server(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(this->get_logger(), "Action server 'execute_task_solution' not available after waiting");
    return false;
  }

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Task execution failed");
    return false;
  }

  return true;
}