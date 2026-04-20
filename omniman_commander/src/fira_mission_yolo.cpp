#include <memory>
#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/point_stamped.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("hello_moveit");

  // YOLO target subscription for letter F
  std::mutex f_mutex;
  bool f_received = false;
  double f_x = 0.0, f_y = 0.0;

  auto f_sub = node->create_subscription<geometry_msgs::msg::PointStamped>(
    "/yolo_target/F", 10,
    [&](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(f_mutex);
      f_x = msg->point.x;
      f_y = msg->point.y;
      f_received = true;
    });

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

  // Move to north pose to view cubes
  {
    move_group_interface.setNamedTarget("north");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (static_cast<bool>(move_group_interface.plan(plan))) {
      draw_title("return to north");
      moveit_visual_tools.trigger();
      if (static_cast<bool>(move_group_interface.execute(plan)))
        RCLCPP_INFO(logger, "Returned to north");
      else
        RCLCPP_ERROR(logger, "to north execute failed");
    } else {
      RCLCPP_ERROR(logger, "to north plan failed");
    }
  }
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // Wait for YOLO target F from /yolo_target/F
  RCLCPP_INFO(logger, "Waiting for 'F' position from /yolo_target/F ...");
  {
    auto start = node->now();
    while (rclcpp::ok() && (node->now() - start).seconds() < 30.0) {
      {
        std::lock_guard<std::mutex> lk(f_mutex);
        if (f_received) break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::lock_guard<std::mutex> lk(f_mutex);
    if (!f_received) {
      RCLCPP_ERROR(logger, "Timed out waiting for 'F' — aborting");
      rclcpp::shutdown();
      spinner.join();
      return 1;
    }
    RCLCPP_INFO(logger, "Got 'F' at base_link (%.3f, %.3f)", f_x, f_y);
  }

  // Copy target position for use in targets
  double pick_x, pick_y;
  {
    std::lock_guard<std::mutex> lk(f_mutex);
    pick_x = f_x;
    pick_y = f_y;
  }

  // Target 1
  {
    geometry_msgs::msg::Pose target;
    target.position.x = pick_x;  target.position.y = pick_y;  target.position.z = 0.10;
    target.orientation.x = 0.9965; target.orientation.y = -0.0128; target.orientation.z = -0.0824; target.orientation.w = 0.0010;
    move_group_interface.setPoseTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto const ok = static_cast<bool>(move_group_interface.plan(plan));
    if (ok) {
      draw_trajectory_tool_path(plan.trajectory_);
      moveit_visual_tools.trigger();
      draw_title("Executing Target 1 (Hover F)");
      moveit_visual_tools.trigger();
      if (static_cast<bool>(move_group_interface.execute(plan)))
        RCLCPP_INFO(logger, "Target 1 executed");
      else
        RCLCPP_ERROR(logger, "Target 1 execute failed");
    } else {
      draw_title("Target 1 Plan Failed!");
      moveit_visual_tools.trigger();
      RCLCPP_ERROR(logger, "Target 1 plan failed");
    }
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
