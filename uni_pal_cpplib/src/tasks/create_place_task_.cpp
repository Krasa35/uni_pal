#include "TaskClient/TaskClient.h"
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

moveit::task_constructor::Task TaskClient::create_place_task_()
{
  moveit::task_constructor::Task task;
  task.stages()->setName("Place Task");

  rclcpp::Node::SharedPtr node = this->shared_from_this();
  task.loadRobotModel(node, "robot_description");
  //   if ((pallet_box_info_.counters.Left.state == PalletBoxInfo::PALLET_STATE::FULL && pallet_box_info_.Side == PalletBoxInfo::PALLET_SIDE::LEFT) || (pallet_box_info_.counters.Right.state == PalletBoxInfo::PALLET_STATE::FULL && pallet_box_info_.Side == PalletBoxInfo::PALLET_SIDE::RIGHT)){
  //     task = createHomingTask(robot_state);
  //     return task;
  //   }
  //   uni_pal_msgs::action::PickPlaceTrajectory::Goal goal;
  //   PalletBoxInfo::PalletCounters cnt = (pallet_box_info_.Side == PalletBoxInfo::PALLET_SIDE::LEFT) ? pallet_box_info_.counters.Left : pallet_box_info_.counters.Right;
  //   goal.box_no   = cnt.Boxes_on_current_layer + 1;
  //   goal.layer_no = cnt.Layers_placed + 1;
  //   goal.pallet_side = (pallet_box_info_.Side == PalletBoxInfo::PALLET_SIDE::IDLE) ? "IDLE":((pallet_box_info_.Side == PalletBoxInfo::PALLET_SIDE::LEFT) ? "LEFT" : "RIGHT");
  //   ActionClient<uni_pal_msgs::action::PickPlaceTrajectory>::send_goal(goal);
  //   do
  //   {
  //     RCLCPP_INFO(LOGGER, "Waiting for action...");
  //     rclcpp::sleep_for(std::chrono::milliseconds(100));
  //   } while (ActionClient<uni_pal_msgs::action::PickPlaceTrajectory>::action_done == false);
  //   rclcpp_action::ClientGoalHandle<uni_pal_msgs::action::PickPlaceTrajectory>::WrappedResult action_result = ActionClient<uni_pal_msgs::action::PickPlaceTrajectory>::get_action_result();
  //   pick_place_trajectory_ = actionResultToPose(action_result);
  //   pallet_box_info_.increment();

  const auto &arm_group_name = static_message_.config.arm_group_name;
  const auto &hand_group_name = static_message_.config.link_names.back();
  const auto &hand_frame = dynamic_message_.actual_pose.current_tcp_pose;

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);
  task.setProperty("timeout", 5.0);

  auto sampling_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node);
  //   auto interpolation_planner = std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();

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

  { // TRAVEL POSITION
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("Go to travel position", sampling_planner);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"group", "ik_frame"});
    stage->setGroup(arm_group_name);
    if (counters_message_.left_pallet.pallet_state == "IN_PROGRESS")
      stage->setGoal("home_left");
    else if (counters_message_.right_pallet.pallet_state == "IN_PROGRESS")
      stage->setGoal("home_right");
    stage->setTimeout(10.0);
    task.add(std::move(stage));
  }

  { // BEFORE PLACE POSITION
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("Placing the " + box_id, cartesian_planner);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"group", "eef", "ik_frame"});
    stage->setGroup(arm_group_name);

    geometry_msgs::msg::PoseStamped before_place;
    before_place.header.stamp = node->now();
    before_place.header.frame_id = "pallet_left";
    before_place.pose = place_pose_.place_pose;
    before_place.pose.position.z += 0.1;
    // before_place.pose.position.x += 0.1 * cos(pallet_box_info_.approach_angle);

    stage->setIKFrame(hand_group_name);
    stage->setGoal(before_place);
    stage->setTimeout(10.0);
    task.add(std::move(stage));
  }
  RCLCPP_INFO(this->get_logger(), "Before place position added");

  {
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("Lower To Pick Position", cartesian_planner);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"group", "ik_frame"});
    stage->setGroup(arm_group_name);
    stage->setIKFrame(hand_frame);
    // stage->setMinMaxDistance(approach_object_min_dist, approach_object_max_dist);
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = -0.1;
    stage->setDirection(vec);
    stage->setTimeout(10.0);
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("detaching " + box_id);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "eef" });
    stage->detachObject(box_id, hand_group_name);
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("Move up", cartesian_planner);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"group", "ik_frame"});
    stage->setGroup(arm_group_name);
    stage->setIKFrame(hand_frame);
    // stage->setMinMaxDistance(approach_object_min_dist, approach_object_max_dist);
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = 0.1;
    stage->setDirection(vec);
    stage->setTimeout(10.0);
    task.add(std::move(stage));
  }

  { // TRAVEL POSITION
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("Go to travel position", sampling_planner);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"group", "ik_frame"});
    stage->setGroup(arm_group_name);
    if (counters_message_.left_pallet.pallet_state == "IN_PROGRESS")
      stage->setGoal("home_left");
    else if (counters_message_.right_pallet.pallet_state == "IN_PROGRESS")
      stage->setGoal("home_right");
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

  //   {// PLACE POSITION
  //     auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("Placing the " + box_id, cartesian_planner);
  //     stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group", "eef" });
  //     stage->setGroup(arm_group_name);
  //     stage->setIKFrame(hand_group_name);
  //     RCLCPP_INFO(LOGGER, printPose(pick_place_trajectory_.place.pose).c_str());
  //     stage->setGoal(pick_place_trajectory_.place);
  //     stage->setTimeout(10.0);
  //     task.add(std::move(stage));
  //   }

  //   {
  //   auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("detaching " + box_id);
  //   stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "eef" });
  //   stage->detachObject(box_id, hand_group_name);
  //   task.add(std::move(stage));
  //   }

  //   {// TRAVEL POSITION
  //     auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("Go to travel position", cartesian_planner);
  //     stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });
  //     stage->setGroup(arm_group_name);
  //     (pallet_box_info_.Side == PalletBoxInfo::PALLET_SIDE::LEFT) ? stage->setGoal("home_left") : stage->setGoal("home_right");
  //     stage->setTimeout(10.0);
  //     task.add(std::move(stage));
  //   }

  //   {
  //     auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("Ensure robot is at home", cartesian_planner);
  //     stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });
  //     stage->setGroup(arm_group_name);
  //     stage->setGoal("home");
  //     stage->setTimeout(10.0);
  //     task.add(std::move(stage));
  //   }

  // {
  //   std::string box_id = "box 0";
  //   auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("Allow collision");
  //   stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {});
  //   stage->allowCollisions(box_id, "gripper", true);
  //   stage->allowCollisions(box_id, "flange", true);
  //   stage->allowCollisions(box_id,
  //                          task.getRobotModel()
  //                              ->getLinkModelNamesWithCollisionGeometry(),
  //                          true);
  //   task.add(std::move(stage));
  // }

  // {
  //   auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("Go to Before Pick Position", cartesian_planner);
  //   stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"group", "ik_frame"});
  //   stage->setGroup(arm_group_name);
  //   stage->setIKFrame(hand_frame);

  //   geometry_msgs::msg::PoseStamped before_pick_pose;
  //   tf2::Quaternion q;
  //   before_pick_pose.header.frame_id = "pickpoint";
  //   bool short_side_leading = pallet_params_message_.programsettings.box_orientation == "SHORT_SIDE_LEADING";
  //   before_pick_pose.pose.position.x = short_side_leading ? pallet_params_message_.box.length / 2000 : pallet_params_message_.box.width / 2000;
  //   before_pick_pose.pose.position.y = short_side_leading ? pallet_params_message_.box.width / 2000 : pallet_params_message_.box.length / 2000;
  //   before_pick_pose.pose.position.z = pallet_params_message_.box.height / 1000 + 0.11;
  //   short_side_leading ? q.setRPY(M_PI, 0, -M_PI_2) : q.setRPY(M_PI, 0, 0);
  //   before_pick_pose.pose.orientation = tf2::toMsg(q);

  //   stage->setGoal(before_pick_pose);
  //   stage->setTimeout(10.0);
  //   task.add(std::move(stage));
  // }

  // {
  //   auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("Lower To Pick Position", cartesian_planner);
  //   stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"group", "ik_frame"});
  //   stage->setGroup(arm_group_name);
  //   stage->setIKFrame(hand_frame);
  //   // stage->setMinMaxDistance(approach_object_min_dist, approach_object_max_dist);
  //   geometry_msgs::msg::Vector3Stamped vec;
  //   vec.header.frame_id = "pickpoint";
  //   vec.vector.z = -0.1;
  //   stage->setDirection(vec);
  //   stage->setTimeout(10.0);
  //   task.add(std::move(stage));
  // }

  // {
  //   std::string box_id = "box 0";
  //   auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("attaching " + box_id);
  //   stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {});
  //   stage->attachObject(box_id, hand_group_name);
  //   task.add(std::move(stage));
  // }

  // {
  //   auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("Move up", cartesian_planner);
  //   stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"group", "ik_frame"});
  //   stage->setGroup(arm_group_name);
  //   stage->setIKFrame(hand_frame);
  //   // stage->setMinMaxDistance(approach_object_min_dist, approach_object_max_dist);
  //   geometry_msgs::msg::Vector3Stamped vec;
  //   vec.header.frame_id = "pickpoint";
  //   vec.vector.z = 0.1;
  //   stage->setDirection(vec);
  //   stage->setTimeout(10.0);
  //   task.add(std::move(stage));
  // }

  return task;
}