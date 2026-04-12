#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "position",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("position");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  auto const pose = move_group_interface.getCurrentPose().pose;

  RCLCPP_INFO(logger, "=== Current End-Effector Pose ===");
  RCLCPP_INFO(logger, "Position    x=%.4f  y=%.4f  z=%.4f",
    pose.position.x, pose.position.y, pose.position.z);
  RCLCPP_INFO(logger, "Quaternion  x=%.4f  y=%.4f  z=%.4f  w=%.4f",
    pose.orientation.x, pose.orientation.y,
    pose.orientation.z, pose.orientation.w);

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
