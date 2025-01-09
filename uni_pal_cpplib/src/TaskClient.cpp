#include "TaskClient/TaskClient.h"

TaskClient::TaskClient(const rclcpp::NodeOptions& options) : Node("task_client", options)
{
    // Subscribers
    static_info_subscriber_ = this->create_subscription<uni_pal_msgs::msg::RobotStaticInfo>(
        "/robot_client/static_info", 10, std::bind(&TaskClient::static_info_subscriber_callback_, this, std::placeholders::_1));
    dynamic_info_subscriber_ = this->create_subscription<uni_pal_msgs::msg::RobotDynamicInfo>(
        "/robot_client/dynamic_info", 10, std::bind(&TaskClient::dynamic_info_subscriber_callback_, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&TaskClient::do_task_, this));
}

// Subscribers
void TaskClient::static_info_subscriber_callback_(const uni_pal_msgs::msg::RobotStaticInfo& msg)
{
    static_message_ = msg;
}

void TaskClient::dynamic_info_subscriber_callback_(const uni_pal_msgs::msg::RobotDynamicInfo& msg)
{
    dynamic_message_ = msg;
}

// MoveIt Task Constructor
void TaskClient::do_task_()
{
    task_ = create_demo_task_();

    try
    {
        task_.init();
    }
    catch (moveit::task_constructor::InitStageException& e)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), e);
        return;
    }

    if (!task_.plan(5))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Task planning failed");
        return;
    }
    task_.introspection().publishSolution(*task_.solutions().front());
    auto action_client = rclcpp_action::create_client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>(shared_from_this(), "execute_task_solution");


    if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server 'execute_task_solution' not available after waiting");
        return;
    }

    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Task execution failed");
        return;
    }

    // auto result = task_.execute(*task_.solutions().front());
    // if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    // {
    //     RCLCPP_ERROR_STREAM(this->get_logger(), "Task execution failed");
    //     return;
    // }

    return;
}