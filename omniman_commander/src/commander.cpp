#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("hello_moveit");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");
  auto gripper_group_interface = MoveGroupInterface(node, "gripper");
  move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group_interface.setPlannerId("PTP");

  RCLCPP_INFO(logger, "Planning Pipeline: %s", move_group_interface.getPlanningPipelineId().c_str());
  RCLCPP_INFO(logger, "Planner ID: %s", move_group_interface.getPlannerId().c_str());

  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XLARGE);
  };
  auto const draw_trajectory_tool_path =
    [&moveit_visual_tools,
     jmg = move_group_interface.getRobotModel()->getJointModelGroup("arm"),
     ee  = move_group_interface.getRobotModel()->getLinkModel("ee_link")](
       auto const trajectory) {
      moveit_visual_tools.publishTrajectoryLine(trajectory, ee, jmg);
    };

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

  // Sync the internal start state with the real-world state before planning.
  // Prevents MoveIt from using a stale "end of previous plan" as the start state.
  move_group_interface.setStartStateToCurrentState();

  // Get the current pose (position + quaternion) of the end effector
  auto current_pose = move_group_interface.getCurrentPose("ee_link").pose;
  RCLCPP_INFO(logger, "Current pose: pos=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)",
    current_pose.position.x, current_pose.position.y, current_pose.position.z,
    current_pose.orientation.x, current_pose.orientation.y,
    current_pose.orientation.z, current_pose.orientation.w);

  // Build target pose from the current pose (same orientation, slight position offset)
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = current_pose.position.x + 0.03;
  target_pose.position.y = current_pose.position.y;
  target_pose.position.z = current_pose.position.z;
  target_pose.orientation = current_pose.orientation;

  RCLCPP_INFO(logger, "Target pose: pos=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)",
    target_pose.position.x, target_pose.position.y, target_pose.position.z,
    target_pose.orientation.x, target_pose.orientation.y,
    target_pose.orientation.z, target_pose.orientation.w);

  // Clear any previous targets/constraints before planning a new motion
  move_group_interface.clearPoseTargets();
  move_group_interface.clearPathConstraints();

  // Follow the MoveIt best-practice pattern from the tutorial:
  //   move_group.setStartStateToCurrentState();
  //   move_group.setPoseTarget(target_pose);
  move_group_interface.setStartStateToCurrentState();
  move_group_interface.setPoseTarget(target_pose, "ee_link");

  // Direct IK check — tests the KDL solver on the target pose in isolation,
  // before planning is even attempted. If this fails, the target is unreachable by IK.
  {
    auto test_state = *move_group_interface.getCurrentState();
    auto const* jmg = test_state.getJointModelGroup("arm");
    bool ik_ok = test_state.setFromIK(jmg, target_pose, "ee_link", 0.1);
    RCLCPP_INFO(logger, "Direct IK test (100ms timeout): %s", ik_ok ? "SUCCESS" : "FAILED");
  }

  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_result = move_group_interface.plan(plan);
  if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
    draw_trajectory_tool_path(plan.trajectory_);
    moveit_visual_tools.trigger();
    draw_title("Executing Target");
    moveit_visual_tools.trigger();
    auto exec_result = move_group_interface.execute(plan);
    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, "Target executed successfully");
    } else {
      RCLCPP_ERROR(logger, "Target execute failed with error code: %d (%s)",
        static_cast<int>(exec_result.val),
        moveit::core::error_code_to_string(exec_result).c_str());
    }
  } else {
    draw_title("Plan Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Plan failed with error code: %d (%s)",
      static_cast<int>(plan_result.val),
      moveit::core::error_code_to_string(plan_result).c_str());

    // Log diagnostics that help identify the failure
    auto joints = move_group_interface.getCurrentJointValues();
    auto names = move_group_interface.getJointNames();
    RCLCPP_ERROR(logger, "Current joint values at failure:");
    for (size_t i = 0; i < joints.size() && i < names.size(); ++i) {
      RCLCPP_ERROR(logger, "  %s = %.4f rad", names[i].c_str(), joints[i]);
    }
    RCLCPP_ERROR(logger, "Target pose was: pos=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)",
      target_pose.position.x, target_pose.position.y, target_pose.position.z,
      target_pose.orientation.x, target_pose.orientation.y,
      target_pose.orientation.z, target_pose.orientation.w);
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
