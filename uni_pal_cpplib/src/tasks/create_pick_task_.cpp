#include "TaskClient/TaskClient.h"
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

moveit::task_constructor::Task TaskClient::create_pick_task_()
{
  moveit::task_constructor::Task task;
  task.stages()->setName("Pick Task");

  rclcpp::Node::SharedPtr node = this->shared_from_this();
  task.loadRobotModel(node, "robot_description");

  const auto &arm_group_name = static_message_.config.arm_group_name;
  const auto &hand_group_name = static_message_.config.link_names.back();
  const auto &hand_frame = dynamic_message_.actual_pose.current_tcp_pose;

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);
  task.setProperty("timeout", 5.0);

  auto sampling_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node);

  auto cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);
  cartesian_planner->setMinFraction(.05);

  std::string box_id = "box " + std::to_string(counters_message_.total_boxes_placed);
  // current state
  {
    auto stage_state_current = std::make_unique<moveit::task_constructor::stages::CurrentState>("current state");
    stage_state_current->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);

    task.add(std::move(stage_state_current));
  }

  {
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("Ensure robot is at home", sampling_planner);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"group"});
    stage->setGroup(arm_group_name);
    stage->setGoal("home");
    stage->setTimeout(10.0);
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("Allow collision");
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {});
    stage->allowCollisions(box_id, "gripper", true);
    stage->allowCollisions(box_id, "flange", true);
    stage->allowCollisions(box_id,
                           task.getRobotModel()
                               ->getLinkModelNamesWithCollisionGeometry(),
                           true);
    for (size_t i = 0; i < counters_message_.total_boxes_placed; i++)
    {
      std::string box_id_prev = "box " + std::to_string(i);
      stage->allowCollisions(box_id, box_id_prev, true);
    }
    
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("Go to Before Pick Position", cartesian_planner);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"group", "ik_frame"});
    stage->setGroup(arm_group_name);
    stage->setIKFrame(hand_frame);

    geometry_msgs::msg::PoseStamped before_pick_pose;
    tf2::Quaternion q;
    before_pick_pose.header.frame_id = "pickpoint";
    bool short_side_leading = pallet_params_message_.programsettings.box_orientation == "SHORT_SIDE_LEADING";
    before_pick_pose.pose.position.x = short_side_leading ? pallet_params_message_.box.length / 2000 : pallet_params_message_.box.width / 2000;
    before_pick_pose.pose.position.y = short_side_leading ? pallet_params_message_.box.width / 2000 : pallet_params_message_.box.length / 2000;
    before_pick_pose.pose.position.z = pallet_params_message_.box.height / 1000 + 0.11;
    short_side_leading ? q.setRPY(M_PI, 0, -M_PI_2) : q.setRPY(M_PI, 0, 0);
    before_pick_pose.pose.orientation = tf2::toMsg(q);

    stage->setGoal(before_pick_pose);
    stage->setTimeout(10.0);
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("Lower To Pick Position", cartesian_planner);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"group", "ik_frame"});
    stage->setGroup(arm_group_name);
    stage->setIKFrame(hand_frame);
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "pickpoint";
    vec.vector.z = -0.1;
    stage->setDirection(vec);
    stage->setTimeout(10.0);
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("attaching " + box_id);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {});
    stage->attachObject(box_id, hand_group_name);
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("Move up", cartesian_planner);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"group", "ik_frame"});
    stage->setGroup(arm_group_name);
    stage->setIKFrame(hand_frame);
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "pickpoint";
    vec.vector.z = 0.1;
    stage->setDirection(vec);
    stage->setTimeout(10.0);
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("Ensure robot is at home", sampling_planner);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"group"});
    stage->setGroup(arm_group_name);
    stage->setGoal("home");
    stage->setTimeout(10.0);
    task.add(std::move(stage));
  }

  return task;
}