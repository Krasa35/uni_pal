#include "TaskClient/TaskClient.h"
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

moveit::task_constructor::Task TaskClient::create_emergency_homing_task_()
{
  moveit::task_constructor::Task task;
  task.stages()->setName("Emergency Homing Task");

  rclcpp::Node::SharedPtr node = this->shared_from_this();
  task.loadRobotModel(node, "robot_description");

  const auto &arm_group_name = static_message_.config.arm_group_name;
  const auto &hand_group_name = dynamic_message_.actual_pose.current_tcp_name;
  const auto &hand_frame = dynamic_message_.actual_pose.current_tcp_pose;

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);
  task.setProperty("timeout", 5.0);

  auto sampling_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node);

  // current state
  {
    auto stage_state_current = std::make_unique<moveit::task_constructor::stages::CurrentState>("current state");
    stage_state_current->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);

    task.add(std::move(stage_state_current));
  }

  {
    auto orientation_stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("go to safe position", sampling_planner);
    orientation_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"group", "eef", "ik_frame", "timeout"});
    orientation_stage->setGroup(arm_group_name);
    orientation_stage->setGoal("safe_position_base_same");
    orientation_stage->setTimeout(10.0);
    task.add(std::move(orientation_stage));

    auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("rotate to super safe position", sampling_planner);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"group", "eef", "ik_frame", "timeout"});
    stage->setGroup(arm_group_name);
    stage->setGoal("safe_up");
    stage->setTimeout(10.0);
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("go to home position", sampling_planner);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"group", "eef", "ik_frame", "timeout"});
    stage->setGroup(arm_group_name);
    stage->setGoal("home");
    stage->setTimeout(10.0);
    task.add(std::move(stage));
  }

  return task;
}