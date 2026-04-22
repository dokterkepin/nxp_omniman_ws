#include <memory>
#include <thread>
#include <algorithm>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;

using moveit::planning_interface::MoveGroupInterface;

struct CommanderParams {
  double arm_max_velocity_scaling_factor = 0.1;
  double arm_max_acceleration_scaling_factor = 0.1;
  std::string end_effector_link = "ee_link";
};

class Commander1 {
public:
  Commander1(
    std::shared_ptr<rclcpp::Logger> logger,
    MoveGroupInterface& arm_move_group,
    MoveGroupInterface& gripper_move_group,
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    CommanderParams& params)
    : logger_(logger),
      arm_move_group_(&arm_move_group),
      gripper_move_group_(&gripper_move_group),
      visual_tools_(&visual_tools),
      params_(&params)
  {
    text_pose_ = Eigen::Isometry3d::Identity();
    text_pose_.translation().z() = 1.0;

    joint_model_group_ = arm_move_group_->getRobotModel()->getJointModelGroup("arm");
  }

  void clear_target_n_constraints() {
    arm_move_group_->clearPoseTargets();
    arm_move_group_->clearPathConstraints();
  }

  bool move_ptp(const geometry_msgs::msg::Pose& target_pose, const std::string& text) {
      std::cout << std::string(125, '*') << std::endl;
      RCLCPP_INFO(*logger_, "Move absolute to `%s`", text.c_str());

      // Clear previous targets and constraints
      clear_target_n_constraints();

      // Set Pilz Industrial Motion Planner with PTP planner for point-to-point motion
      arm_move_group_->setPlannerId("PTP");

      // Set velocity scaling for this motion
      arm_move_group_->setMaxVelocityScalingFactor(params_->arm_max_velocity_scaling_factor);
      arm_move_group_->setMaxAccelerationScalingFactor(params_->arm_max_acceleration_scaling_factor);

      // Set the start state to the current state
      auto current_state = arm_move_group_->getCurrentState();
      arm_move_group_->setStartState(*current_state);
      arm_move_group_->setPoseTarget(target_pose, params_->end_effector_link);  // planning in cartesian space

      std::string text_lower = text;
      std::transform(text_lower.begin(), text_lower.end(), text_lower.begin(), ::tolower);

      // Plan the motion
      auto plan_result = arm_move_group_->plan(plan_);
      is_plan_success_ = plan_result == moveit::core::MoveItErrorCode::SUCCESS;
      RCLCPP_INFO(*logger_, "Move PTP Planning: %s", is_plan_success_ ? "succeeded" : "failed");

      // Visualize the target pose and trajectory
      visual_tools_->publishAxisLabeled(target_pose, text_lower);
      visual_tools_->publishText(text_pose_, text, rvt::WHITE, rvt::XLARGE);

      is_execution_success_ = false;
      if (is_plan_success_) {
          visual_tools_->publishTrajectoryLine(plan_.trajectory_, joint_model_group_);
          visual_tools_->trigger();

          // Use execute instead of move to ensure we use the validated plan
          auto execution_result = arm_move_group_->execute(plan_);
          is_execution_success_ = execution_result == moveit::core::MoveItErrorCode::SUCCESS;
          RCLCPP_INFO(*logger_, "PTP trajectory execution: %s", is_execution_success_ ? "succeeded" : "failed");

          if (!is_execution_success_) {
              RCLCPP_ERROR(*logger_, "PTP trajectory execution failed with error code: %d (%s)", static_cast<int>(execution_result.val),
                          moveit::core::error_code_to_string(execution_result).c_str());
          }
      } else {
          RCLCPP_WARN(*logger_, "PTP path planning failed with error code: %d (%s)", static_cast<int>(plan_result.val),
                      moveit::core::error_code_to_string(plan_result).c_str());
      }

      return is_plan_success_ && is_execution_success_;
  }

private:
  std::shared_ptr<rclcpp::Logger> logger_;
  MoveGroupInterface* arm_move_group_;
  MoveGroupInterface* gripper_move_group_;
  moveit_visual_tools::MoveItVisualTools* visual_tools_;
  CommanderParams* params_;

  MoveGroupInterface::Plan plan_;
  bool is_plan_success_ = false;
  bool is_execution_success_ = false;
  Eigen::Isometry3d text_pose_;
  const moveit::core::JointModelGroup* joint_model_group_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "commander1",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("commander1");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  auto move_group_interface = MoveGroupInterface(node, "arm");
  auto gripper_group_interface = MoveGroupInterface(node, "gripper");
  move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group_interface.setPlannerId("PTP");
  RCLCPP_INFO(logger, "Planning Pipeline: %s", move_group_interface.getPlanningPipelineId().c_str());
  RCLCPP_INFO(logger, "Planner ID: %s", move_group_interface.getPlannerId().c_str());

  auto visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel()};
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // Add floor collision object
  {
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::msg::CollisionObject floor;
    floor.header.frame_id = move_group_interface.getPlanningFrame();
    floor.id = "floor";
    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {2.0, 2.0, 0.01};
    geometry_msgs::msg::Pose floor_pose;
    floor_pose.orientation.w = 1.0;
    floor_pose.position.z = -0.01;
    floor.primitives.push_back(box);
    floor.primitive_poses.push_back(floor_pose);
    floor.operation = floor.ADD;
    planning_scene_interface.applyCollisionObject(floor);
    RCLCPP_INFO(logger, "Floor collision object added");
  }

  RCLCPP_INFO(logger, "Default EE link: %s", move_group_interface.getEndEffectorLink().c_str());
  RCLCPP_INFO(logger, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "Pose reference frame: %s", move_group_interface.getPoseReferenceFrame().c_str());

  auto current_pose = move_group_interface.getCurrentPose("ee_link").pose;
  RCLCPP_INFO(logger, "Current pose (ee_link): pos=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)",
    current_pose.position.x, current_pose.position.y, current_pose.position.z,
    current_pose.orientation.x, current_pose.orientation.y,
    current_pose.orientation.z, current_pose.orientation.w);

  auto default_pose = move_group_interface.getCurrentPose().pose;
  RCLCPP_INFO(logger, "Current pose (default): pos=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)",
    default_pose.position.x, default_pose.position.y, default_pose.position.z,
    default_pose.orientation.x, default_pose.orientation.y,
    default_pose.orientation.z, default_pose.orientation.w);

  CommanderParams params;
  auto logger_ptr = std::make_shared<rclcpp::Logger>(logger);
  Commander1 commander(logger_ptr, move_group_interface, gripper_group_interface, visual_tools, params);

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = current_pose.position.x + 0.03;
  target_pose.position.y = current_pose.position.y;
  target_pose.position.z = current_pose.position.z;
  target_pose.orientation.x = current_pose.orientation.x;
  target_pose.orientation.y = current_pose.orientation.y;
  target_pose.orientation.z = current_pose.orientation.z;
  target_pose.orientation.w = current_pose.orientation.w;
  RCLCPP_INFO(logger, "Target pose: pos=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)",
    target_pose.position.x, target_pose.position.y, target_pose.position.z,
    target_pose.orientation.x, target_pose.orientation.y,
    target_pose.orientation.z, target_pose.orientation.w);
  commander.move_ptp(target_pose, "CurrentPose");

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
