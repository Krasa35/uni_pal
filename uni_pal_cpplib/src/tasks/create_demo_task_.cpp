#include "TaskClient/TaskClient.h"
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

moveit::task_constructor::Task TaskClient::create_demo_task_()
{
    moveit::task_constructor::Task task;
    task.stages()->setName("Demo Task");

    rclcpp::Node::SharedPtr node = this->shared_from_this();
    task.loadRobotModel(node, "robot_description");

    // Create a stage to get the current state
    auto current_state = std::make_unique<moveit::task_constructor::stages::CurrentState>("current state");
    task.add(std::move(current_state));

    // Create a planner
    auto pipeline_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node);

    // Create a stage to move to a named target
    auto move_to = std::make_unique<moveit::task_constructor::stages::MoveTo>("move to home", pipeline_planner);
    move_to->setGroup("ur_manipulator");
    move_to->setGoal("home");
    task.add(std::move(move_to));

    return task;
}