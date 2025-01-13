#include "TaskClient/TaskClient.h"
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

moveit::task_constructor::Task TaskClient::create_pick_place_task_()
{
  moveit::task_constructor::Task task;
  task.stages()->setName("Pick Task");

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
  const auto &hand_group_name = dynamic_message_.actual_pose.current_tcp_name;
  const auto &hand_frame = dynamic_message_.actual_pose.current_tcp_pose;

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_group_name);
  task.setProperty("timeout", 5.0);

  //   // Disable warnings for this line, as it's a variable that's set but not used in this example
  //   #pragma GCC diagnostic push
  //   #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  //   moveit::task_constructor::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  //   #pragma GCC diagnostic pop

  //   auto sampling_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node_);
  //   auto interpolation_planner = std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();

  //   auto cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
  //   cartesian_planner->setMaxVelocityScalingFactor(1.0);
  //   cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  //   cartesian_planner->setStepSize(.01);

  // current state
  {
    auto stage_state_current = std::make_unique<moveit::task_constructor::stages::CurrentState>("current state");
    stage_state_current->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);

    task.add(std::move(stage_state_current));
  }

  // { // Add box task
  //   auto stage_add_box = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("generate object");
  //   moveit_msgs::msg::CollisionObject box = create_box_(box_id);
  //   stage_add_box->addObject(box);
  //   task.add(std::move(stage_add_box));
  // }

  //   {
  //     auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("Ensure robot is at home", cartesian_planner);
  //     stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });
  //     stage->setGroup(arm_group_name);
  //     stage->setGoal("home");
  //     stage->setTimeout(10.0);
  //     task.add(std::move(stage));
  //   }

  //   {
  //     auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("Go to Pick position", cartesian_planner);
  //     stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group","eef" });
  //     stage->setGroup(arm_group_name);
  //     stage->setIKFrame(hand_group_name);
  //     stage->setGoal(pick_place_trajectory_.pick);
  //     stage->setTimeout(10.0);
  //     task.add(std::move(stage));
  //   }

  //   {
  //   auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("attaching " + box_id);
  //   stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "eef" });
  //   stage->attachObject(box_id, hand_group_name);
  //   // stage->allowCollisions(box_id,
  //   //                         task.getRobotModel()
  //   //                         ->getJointModelGroup(hand_group_name)
  //   //                         ->getLinkModelNamesWithCollisionGeometry(),
  //   //                         false);
  //   stage->allowCollisions(box_id,
  //                           task.getRobotModel()
  //                           ->getJointModelGroup(arm_group_name)
  //                           ->getLinkModelNamesWithCollisionGeometry(),
  //                           false);
  //   task.add(std::move(stage));
  //   }

  //   {
  //     auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("Ensure robot is at home", cartesian_planner);
  //     stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });
  //     stage->setGroup(arm_group_name);
  //     stage->setGoal("home");
  //     stage->setTimeout(10.0);
  //     task.add(std::move(stage));
  //   }

  //   {// TRAVEL POSITION
  //     auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("Go to travel position", cartesian_planner);
  //     stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });
  //     stage->setGroup(arm_group_name);
  //     (pallet_box_info_.Side == PalletBoxInfo::PALLET_SIDE::LEFT) ? stage->setGoal("home_left") : stage->setGoal("home_right");
  //     stage->setTimeout(10.0);
  //     task.add(std::move(stage));
  //   }

  //   {// BEFORE PLACE POSITION
  //     auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("Placing the " + box_id, cartesian_planner);
  //     stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group", "eef" });
  //     stage->setGroup(arm_group_name);

  //     geometry_msgs::msg::PoseStamped before_place = pick_place_trajectory_.place;
  //     before_place.pose.position.z += pallet_box_info_.box.height + 0.05;// * sin(pallet_box_info_.approach_angle);

  //     // before_place.pose.position.x += 0.1 * cos(pallet_box_info_.approach_angle);

  //     stage->setIKFrame(hand_group_name);
  //     RCLCPP_INFO(LOGGER, printPose(before_place.pose).c_str());
  //     stage->setGoal(before_place);
  //     stage->setTimeout(10.0);
  //     task.add(std::move(stage));
  //   }

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

  return task;
}