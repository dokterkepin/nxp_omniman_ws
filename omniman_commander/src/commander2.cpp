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
  RCLCPP_INFO(logger, "Gripper Planning Pipeline: %s", gripper_group_interface.getPlanningPipelineId().c_str());


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

  for (int i = 0; i < 100; ++i) {
    RCLCPP_INFO(logger, "=== Iteration %d/100 ===", i + 1);

    // Target 1
    {
      geometry_msgs::msg::Pose target;
      target.position.x = 0.1919;  target.position.y = -0.0436;  target.position.z = 0.0646;
      target.orientation.x = 0.0846; target.orientation.y = 0.09961; target.orientation.z = -0.0179; target.orientation.w = 0.0172;
      move_group_interface.setPoseTarget(target);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto const ok = static_cast<bool>(move_group_interface.plan(plan));
      if (ok) {
        draw_trajectory_tool_path(plan.trajectory_);
        moveit_visual_tools.trigger();
        draw_title("Executing Target 1");
        moveit_visual_tools.trigger();
        if (static_cast<bool>(move_group_interface.execute(plan)))
          RCLCPP_INFO(logger, "Target 1 executed");
        else
          RCLCPP_ERROR(logger, "[iter %d] Target 1 execute failed", i + 1);
      } else {
        draw_title("Target 1 Plan Failed!");
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(logger, "[iter %d] Target 1 plan failed", i + 1);
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Target 2
    {
      geometry_msgs::msg::Pose target;
      target.position.x = 0.1919;  target.position.y = 0.0436;  target.position.z = 0.0646;
      target.orientation.x = 0.0846; target.orientation.y = 0.09961; target.orientation.z = -0.0179; target.orientation.w = 0.0172;
      move_group_interface.setPoseTarget(target);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto const ok = static_cast<bool>(move_group_interface.plan(plan));
      if (ok) {
        draw_trajectory_tool_path(plan.trajectory_);
        moveit_visual_tools.trigger();
        draw_title("Executing Target 2");
        moveit_visual_tools.trigger();
        if (static_cast<bool>(move_group_interface.execute(plan)))
          RCLCPP_INFO(logger, "Target 2 executed");
        else
          RCLCPP_ERROR(logger, "[iter %d] Target 2 execute failed", i + 1);
      } else {
        draw_title("Target 2 Plan Failed!");
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(logger, "[iter %d] Target 2 plan failed", i + 1);
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Target 3
    {
      geometry_msgs::msg::Pose target;
      target.position.x = 0.1752;  target.position.y = 0.1319;  target.position.z = 0.0668;
      target.orientation.x = 0.7522; target.orientation.y = 0.6584; target.orientation.z = 0.0063; target.orientation.w = 0.0254;
      move_group_interface.setPoseTarget(target);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto const ok = static_cast<bool>(move_group_interface.plan(plan));
      if (ok) {
        draw_trajectory_tool_path(plan.trajectory_);
        moveit_visual_tools.trigger();
        draw_title("Executing Target 3");
        moveit_visual_tools.trigger();
        if (static_cast<bool>(move_group_interface.execute(plan)))
          RCLCPP_INFO(logger, "Target 3 executed");
        else
          RCLCPP_ERROR(logger, "[iter %d] Target 3 execute failed", i + 1);
      } else {
        draw_title("Target 3 Plan Failed!");
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(logger, "[iter %d] Target 3 plan failed", i + 1);
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));

  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
   
} 