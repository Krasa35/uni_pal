#include "TaskClient/TaskClient.h"
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

moveit::task_constructor::Task TaskClient::create_homing_task_()
{
    moveit::task_constructor::Task task;
    task.stages()->setName("Homing Task");

    rclcpp::Node::SharedPtr node = this->shared_from_this();
    task.loadRobotModel(node, "robot_description");

    const auto& arm_group_name = static_message_.config.arm_group_name;
    const auto& hand_group_name = dynamic_message_.actual_pose.current_tcp_name;
    const auto& hand_frame = dynamic_message_.actual_pose.current_tcp_pose;

    // Set task properties
    task.setProperty("group", arm_group_name);
    task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", hand_frame);
    task.setProperty("timeout", 5.0);

    auto sampling_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node);
    auto interpolation_planner = std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();

    auto cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.001);
    cartesian_planner->setMinFraction(.05);

    // current state
    {
        auto stage_state_current = std::make_unique<moveit::task_constructor::stages::CurrentState>("current state");
        stage_state_current->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);

        task.add(std::move(stage_state_current)); 
    }

    // safe positionse
    if ((dynamic_message_.state.hyper_extension && dynamic_message_.state.high_above_base) || dynamic_message_.state.hyper_roll)
    {
      auto orientation_stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("go to safe position", sampling_planner);
      orientation_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group", "eef", "ik_frame", "timeout" });
      orientation_stage->setGroup(arm_group_name);
      orientation_stage->setGoal("safe_position_base_same");
      orientation_stage->setTimeout(10.0);
      task.add(std::move(orientation_stage));

      auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("rotate to super safe position", sampling_planner);
      stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group", "eef", "ik_frame", "timeout" });
      stage->setGroup(arm_group_name);
      stage->setGoal("safe_up");
      stage->setTimeout(10.0);
      task.add(std::move(stage));
    }
    else
    {
     // back off
     if (!dynamic_message_.state.over_base){
      {
        auto back_off_stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("back off", interpolation_planner);
        back_off_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group", "eef", "ik_frame", "timeout" });
        geometry_msgs::msg::Vector3Stamped vec;
        // vec.header.frame_id = dynamic_message_.data.tcp.first;
        vec.header.frame_id = "base";
        if (!(dynamic_message_.state.hyper_extension || dynamic_message_.state.hyper_flexion || dynamic_message_.state.forearm_gripper_warn))
        {
          back_off_stage->setMinMaxDistance(0.1, 0.15);
          vec.vector.x = 0;
          vec.vector.y = 0;
          vec.vector.z = 1;
        }
        else{
          back_off_stage->setMinMaxDistance(0.1, 0.25);
        //   int direction = (dynamic_message_.state.hyper_flexion) ? 1 : -1;
          vec.vector.x = 0;
          vec.vector.y = 0;
          vec.vector.z = 1;
        //   vec.vector.x = dynamic_message_.actual_pose.vec_to_base.x() * direction;
        //   vec.vector.y = dynamic_message_.actual_pose.vec_to_base.y() * direction;
        //   vec.vector.z = (dynamic_message_.state.high_above_base) ? 0.15 : ((dynamic_message_.state.hyper_extension) ? 0.5 : 1);
        }
        back_off_stage->setDirection(vec);
        task.add(std::move(back_off_stage));
      }

       // safe orientation
        auto orientation_stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("go to safe orientation", cartesian_planner);
        orientation_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group", "eef", "ik_frame", "timeout" });
        orientation_stage->setGroup(arm_group_name);
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "base";
        pose.pose = dynamic_message_.actual_pose.pose_in_base;
        pose.pose.position.x += (dynamic_message_.state.over_left_pallet) ? 0.3 : -0.3;
        pose.pose.position.z += 0.1;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = -0.707;
        pose.pose.orientation.w = 0.707;


        orientation_stage->setGoal(pose);
        // orientation_stage->setGoal("safe_orientation");
        orientation_stage->setTimeout(10.0);
        task.add(std::move(orientation_stage));
      }
    }

    // home_right or home_left - over the pallet
    {
      if (!((dynamic_message_.state.hyper_extension && dynamic_message_.state.high_above_base) || dynamic_message_.state.hyper_roll))
      {
        // right pallet
        if (dynamic_message_.state.over_right_pallet)
        {
        auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("go to home over right pallet", sampling_planner);
        stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group", "eef", "ik_frame", "timeout" });
        stage->setGroup(arm_group_name);
        stage->setGoal("home_right");
        stage->setTimeout(10.0);
        task.add(std::move(stage));
        }
        // left pallet
        else if (dynamic_message_.state.over_left_pallet)
        {
        auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("go to home over left pallet", sampling_planner);
        stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group", "eef", "ik_frame", "timeout" });
        stage->setGroup(arm_group_name);
        stage->setGoal("home_left");
        stage->setTimeout(10.0);
        task.add(std::move(stage));
        }
      }
    }

    // home - over the conveyor
    {
        auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("go to home position", sampling_planner);
        stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group", "eef", "ik_frame", "timeout" });
        stage->setGroup(arm_group_name);
        stage->setGoal("home");
        stage->setTimeout(10.0);
        task.add(std::move(stage));
    }

  return task;
}