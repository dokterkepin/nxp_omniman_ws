#ifndef OMNIMAN_COMMANDER_ARM_COMMANDER_HPP
#define OMNIMAN_COMMANDER_ARM_COMMANDER_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

namespace omniman
{

// Position + orientation offset relative to a reference pose.
//   dx/dy/dz         : position offset in meters
//   droll/dpitch/dyaw: orientation offset in radians (applied in the reference's local frame)
struct Offset
{
  double dx = 0.0, dy = 0.0, dz = 0.0;
  double droll = 0.0, dpitch = 0.0, dyaw = 0.0;
};

// Build a pose by adding an offset to a reference pose: position adds (dx,dy,dz);
// orientation is the reference orientation composed with the RPY offset in the
// reference's local frame. All-zero offset returns the reference pose unchanged.
geometry_msgs::msg::Pose apply_offset(const geometry_msgs::msg::Pose & reference,
                                      const Offset & offset);

// Thin reusable wrapper over MoveIt's MoveGroupInterface for the arm + gripper.
// Mission nodes (pick-and-place, ArUco, etc.) construct one of these and call its
// methods; the motion/planning details live here, not in every mission file.
class ArmCommander
{
public:
  ArmCommander(const rclcpp::Node::SharedPtr & node,
               const std::string & arm_group = "arm",
               const std::string & gripper_group = "gripper");

  // Select which planning pipeline + planner the arm uses (e.g.
  // "pilz_industrial_motion_planner" + "PTP"). Stored and re-applied on every
  // move_to_pose() call so no other code can silently override it.
  void set_planner(const std::string & pipeline_id, const std::string & planner_id);

  // Set velocity and acceleration scaling factors (0.0–1.0) applied on every
  // move_to_pose() call. Loaded from poses.yaml so no recompile needed.
  void set_scaling(double velocity_scale, double acceleration_scale);

  // Plan + execute the arm to a Cartesian pose. The planner solves IK internally.
  // `label` is used only for log messages. Returns false on plan or execute failure.
  bool move_to_pose(const geometry_msgs::msg::Pose & target, const std::string & label);

  // Plan + execute the arm to a named SRDF group state (e.g. "ready").
  bool move_named(const std::string & named_target);

  // Plan + execute the gripper to a named SRDF group state (e.g. "open" / "close").
  bool move_gripper(const std::string & named_target);

  // Current end-effector pose, using the group's default end-effector link.
  geometry_msgs::msg::Pose current_pose();

  // Add a large floor collision object so plans don't route through the table.
  void add_floor();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  moveit::planning_interface::MoveGroupInterface arm_;
  moveit::planning_interface::MoveGroupInterface gripper_;
  std::string pipeline_id_;
  std::string planner_id_;
  double velocity_scale_     = 1.0;
  double acceleration_scale_ = 1.0;
};

}  // namespace omniman

#endif  // OMNIMAN_COMMANDER_ARM_COMMANDER_HPP
