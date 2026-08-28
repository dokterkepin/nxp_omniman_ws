#include "omniman_commander/arm_commander.hpp"

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>

namespace omniman {

geometry_msgs::msg::Pose apply_offset(const geometry_msgs::msg::Pose& reference,
                                      const Offset& offset) {
  geometry_msgs::msg::Pose msg = reference;
  msg.position.x += offset.dx;
  msg.position.y += offset.dy;
  msg.position.z += offset.dz;

  tf2::Quaternion q_ref(reference.orientation.x, reference.orientation.y, reference.orientation.z,
                        reference.orientation.w);
  tf2::Quaternion q_off;
  q_off.setRPY(offset.droll, offset.dpitch, offset.dyaw);
  tf2::Quaternion q_target = q_ref * q_off;
  q_target.normalize();

  msg.orientation.x = q_target.x();
  msg.orientation.y = q_target.y();
  msg.orientation.z = q_target.z();
  msg.orientation.w = q_target.w();
  return msg;
}

ArmCommander::ArmCommander(const rclcpp::Node::SharedPtr& node, const std::string& arm_group,
                           const std::string& gripper_group)
    : node_(node),
      logger_(node->get_logger()),
      arm_(node, arm_group),
      gripper_(node, gripper_group) {
  RCLCPP_INFO(logger_, "ArmCommander ready for groups [%s] / [%s]", arm_group.c_str(),
              gripper_group.c_str());
}

void ArmCommander::set_planner(const std::string& pipeline_id, const std::string& planner_id) {
  pipeline_id_ = pipeline_id;
  planner_id_ = planner_id;
  arm_.setPlanningPipelineId(pipeline_id_);
  arm_.setPlannerId(planner_id_);
  RCLCPP_INFO(logger_, "Planner selected — pipeline: %s, planner: %s",
              arm_.getPlanningPipelineId().c_str(), arm_.getPlannerId().c_str());
}

void ArmCommander::set_scaling(double velocity_scale, double acceleration_scale) {
  velocity_scale_ = velocity_scale;
  acceleration_scale_ = acceleration_scale;
  RCLCPP_INFO(logger_, "Scaling set — velocity: %.2f, acceleration: %.2f", velocity_scale_,
              acceleration_scale_);
}

bool ArmCommander::move_to_pose(const geometry_msgs::msg::Pose& target, const std::string& label) {
  RCLCPP_INFO(logger_, "[%s] pos=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)", label.c_str(),
              target.position.x, target.position.y, target.position.z, target.orientation.x,
              target.orientation.y, target.orientation.z, target.orientation.w);

  // Re-apply planner and scaling every call — prevents any stale override.
  arm_.setPlanningPipelineId(pipeline_id_);
  arm_.setPlannerId(planner_id_);
  arm_.setMaxVelocityScalingFactor(velocity_scale_);
  arm_.setMaxAccelerationScalingFactor(acceleration_scale_);

  // Sync the internal start state with the real robot and clear any stale goals.
  arm_.setStartStateToCurrentState();
  arm_.clearPoseTargets();
  arm_.clearPathConstraints();
  arm_.setStartStateToCurrentState();

  // Cartesian pose goal — the planner solves IK while planning.
  arm_.setPoseTarget(target, "ee_link");

  // Direct IK check (diagnostic only, does not affect planning): separates
  // "unreachable" from "plan failed" in the logs.
  RCLCPP_INFO(logger_, "  Direct IK check (5ms, ee_link): %s",
              is_reachable(target) ? "reachable" : "NO SOLUTION");

  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_result = arm_.plan(plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger_, "  [%s] planning failed: code %d (%s)", label.c_str(),
                 static_cast<int>(plan_result.val),
                 moveit::core::errorCodeToString(plan_result).c_str());
    auto joints = arm_.getCurrentJointValues();
    auto names = arm_.getJointNames();
    for (size_t i = 0; i < joints.size() && i < names.size(); ++i) {
      RCLCPP_ERROR(logger_, "    %s = %.4f rad", names[i].c_str(), joints[i]);
    }
    return false;
  }

  // Execute
  auto exec_result = arm_.execute(plan);
  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger_, "  [%s] execution failed: code %d (%s)", label.c_str(),
                 static_cast<int>(exec_result.val),
                 moveit::core::errorCodeToString(exec_result).c_str());
    return false;
  }
  RCLCPP_INFO(logger_, "  [%s] done", label.c_str());
  return true;
}

bool ArmCommander::is_reachable(const geometry_msgs::msg::Pose& target) {
  auto test_state = *arm_.getCurrentState();
  auto const* jmg = test_state.getJointModelGroup(arm_.getName());
  // Same link ("ee_link") and same 5ms timeout as the planner (kinematics.yaml),
  // so "reachable" actually predicts whether the plan will succeed.
  return test_state.setFromIK(jmg, target, "ee_link", 0.005);
}

bool ArmCommander::move_to_first_reachable(const std::vector<geometry_msgs::msg::Pose>& candidates,
                                           const std::string& label) {
  for (size_t i = 0; i < candidates.size(); ++i) {
    if (!is_reachable(candidates[i])) {
      RCLCPP_WARN(logger_, "[%s] candidate %zu/%zu: NO IK — skipping", label.c_str(), i + 1,
                  candidates.size());
      continue;
    }
    RCLCPP_INFO(logger_, "[%s] candidate %zu/%zu reachable — planning", label.c_str(), i + 1,
                candidates.size());
    if (move_to_pose(candidates[i], label)) {
      return true;
    }
    RCLCPP_WARN(logger_, "[%s] candidate %zu/%zu plan/exec failed — trying next", label.c_str(),
                i + 1, candidates.size());
  }
  RCLCPP_ERROR(logger_, "[%s] no usable candidate among %zu", label.c_str(), candidates.size());
  return false;
}

bool ArmCommander::move_named(const std::string& named_target) {
  RCLCPP_INFO(logger_, "Arm -> named target [%s]", named_target.c_str());
  arm_.setNamedTarget(named_target);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (arm_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger_, "  Arm plan failed for [%s]", named_target.c_str());
    return false;
  }
  if (arm_.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger_, "  Arm execute failed for [%s]", named_target.c_str());
    return false;
  }
  return true;
}

bool ArmCommander::move_gripper(const std::string& named_target) {
  RCLCPP_INFO(logger_, "Gripper -> %s", named_target.c_str());
  gripper_.setNamedTarget(named_target);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (gripper_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger_, "  Gripper plan failed for [%s]", named_target.c_str());
    return false;
  }
  if (gripper_.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger_, "  Gripper execute failed for [%s]", named_target.c_str());
    return false;
  }
  return true;
}

geometry_msgs::msg::Pose ArmCommander::current_pose() {
  // Use the same link we plan to ("ee_link") so the captured pose matches the
  // frame used by move_to_pose / setPoseTarget.
  return arm_.getCurrentPose("ee_link").pose;
}

void ArmCommander::add_floor() {
  moveit::planning_interface::PlanningSceneInterface psi;
  moveit_msgs::msg::CollisionObject floor;
  floor.header.frame_id = arm_.getPlanningFrame();
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
  psi.applyCollisionObject(floor);
  RCLCPP_INFO(logger_, "Floor collision object added");
}

}  // namespace omniman
