/*******************************************************************************
 *  omniman_pose_tracking_node.cpp
 *
 *  Single-arm MoveIt Servo pose tracking for the omniman 6-DOF arm.
 *
 *  Subscribes to a target end-effector pose (default /hand_target_pose, a
 *  geometry_msgs/PoseStamped expressed in the servo planning frame) and uses
 *  moveit_servo::PoseTracking to servo the arm end-effector toward it in real
 *  time.  The gripper is handled separately by the MediaPipe Python node.
 *
 *  Adapted from the dual-arm MediaPipe demo (Nabil-Miri/mediapipe_dual_arm_control)
 *  for the single omniman arm.  Differences from the original:
 *    - one arm only (no left/right, no namespaces)
 *    - no gripper publisher here (Python sends a GripperCommand action goal)
 *    - PoseTracking owns its own Servo, so we do NOT create a second one
 *    - position-only tracking by default (the 6-DOF arm has no null space, so
 *      tracking hand orientation tends to drag the EE position around -- see
 *      track_orientation param)
 *
 * BSD 3-Clause License (inherited from moveit_servo)
 *******************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <thread>
#include <atomic>
#include <cmath>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("omniman_pose_tracking_node");

// Logs moveit_servo status whenever it changes.
class StatusMonitor
{
public:
  StatusMonitor(const rclcpp::Node::SharedPtr& node, const std::string& topic)
  {
    sub_ = node->create_subscription<std_msgs::msg::Int8>(
        topic, rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::Int8::ConstSharedPtr& msg) { return statusCB(msg); });
  }

private:
  void statusCB(const std_msgs::msg::Int8::ConstSharedPtr& msg)
  {
    moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest_status != status_)
    {
      status_ = latest_status;
      const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
      RCLCPP_INFO_STREAM(LOGGER, "Servo status: " << status_str);
    }
  }

  moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("omniman_pose_tracking_node");

  // ---- Parameters (all tunable from the launch file) ----------------------
  const std::string hand_pose_topic = node->declare_parameter<std::string>("hand_pose_topic", "/hand_target_pose");
  const double lin_tolerance = node->declare_parameter<double>("linear_tolerance", 0.01);     // [m]
  const double rot_tolerance = node->declare_parameter<double>("rotational_tolerance", 0.1);  // [rad]
  const double target_pose_timeout = node->declare_parameter<double>("target_pose_timeout", 0.5);  // [s] per moveToPose call
  const double no_pose_timeout = node->declare_parameter<double>("no_pose_timeout", 1.0);     // [s] stop tracking if idle
  const bool track_orientation = node->declare_parameter<bool>("track_orientation", false);
  // Generous safety box (planning frame, metres). The Python node already
  // clamps to the reachable workspace; this just rejects NaN / absurd targets.
  const double ws_xy_limit = node->declare_parameter<double>("workspace_xy_limit", 2.0);
  const double ws_z_limit = node->declare_parameter<double>("workspace_z_limit", 2.0);

  RCLCPP_INFO(LOGGER, "Starting omniman single-arm pose tracking node");
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  std::thread executor_thread([&executor]() { executor->spin(); });

  // ---- Servo parameters ---------------------------------------------------
  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node);
  if (servo_parameters == nullptr)
  {
    RCLCPP_FATAL(LOGGER, "Could not get servo parameters! Check the launch file loads the omniman servo config.");
    return EXIT_FAILURE;
  }

  // ---- Planning scene monitor --------------------------------------------
  auto planning_scene_monitor =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    RCLCPP_FATAL(LOGGER, "Error setting up the PlanningSceneMonitor.");
    return EXIT_FAILURE;
  }
  planning_scene_monitor->providePlanningSceneService();
  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC, false);
  planning_scene_monitor->startStateMonitor(servo_parameters->joint_topic);
  planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

  if (!planning_scene_monitor->waitForCurrentRobotState(node->now(), 5.0))
  {
    RCLCPP_FATAL(LOGGER, "Timed out waiting for /joint_states. Is the robot (or sim) publishing joint states?");
    return EXIT_FAILURE;
  }

  // ---- Pose tracker (creates and starts its own Servo internally) ---------
  moveit_servo::PoseTracking tracker(node, servo_parameters, planning_scene_monitor);

  RCLCPP_INFO(LOGGER, "=== OMNIMAN SERVO CONFIG ===");
  RCLCPP_INFO(LOGGER, "Move group       : %s", servo_parameters->move_group_name.c_str());
  RCLCPP_INFO(LOGGER, "Planning frame   : %s", servo_parameters->planning_frame.c_str());
  RCLCPP_INFO(LOGGER, "EE frame         : %s", servo_parameters->ee_frame_name.c_str());
  RCLCPP_INFO(LOGGER, "Command out topic: %s", servo_parameters->command_out_topic.c_str());
  RCLCPP_INFO(LOGGER, "Track orientation: %s", track_orientation ? "true" : "false (position-only)");
  RCLCPP_INFO(LOGGER, "============================");

  // Republish target poses onto the tracker's "target_pose" topic.
  auto target_pose_pub =
      node->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", rclcpp::SystemDefaultsQoS());

  StatusMonitor status_monitor(node, servo_parameters->status_topic);

  const Eigen::Vector3d lin_tol{ lin_tolerance, lin_tolerance, lin_tolerance };

  // Capture the current EE orientation so position-only mode can hold it.
  geometry_msgs::msg::TransformStamped current_ee_tf;
  tracker.getCommandFrameTransform(current_ee_tf);
  const geometry_msgs::msg::Quaternion held_orientation = current_ee_tf.transform.rotation;
  RCLCPP_INFO(LOGGER, "Holding EE orientation [x=%.3f y=%.3f z=%.3f w=%.3f] in position-only mode",
              held_orientation.x, held_orientation.y, held_orientation.z, held_orientation.w);

  std::atomic<bool> tracking_active{ false };
  std::shared_ptr<std::thread> move_to_pose_thread;
  rclcpp::Time last_pose_time = node->now();
  bool pose_received = false;

  // ---- Incoming target-pose subscription ----------------------------------
  auto hand_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      hand_pose_topic, 10, [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        const double x = msg->pose.position.x;
        const double y = msg->pose.position.y;
        const double z = msg->pose.position.z;

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
            std::abs(x) > ws_xy_limit || std::abs(y) > ws_xy_limit || std::abs(z) > ws_z_limit)
        {
          RCLCPP_WARN_THROTTLE(LOGGER, *node->get_clock(), 2000,
                               "Target outside safety box, ignoring: [%.3f, %.3f, %.3f]", x, y, z);
          return;
        }

        auto relay_msg = *msg;
        relay_msg.header.stamp = node->now();
        if (!track_orientation)
          relay_msg.pose.orientation = held_orientation;  // position-only tracking
        target_pose_pub->publish(relay_msg);

        pose_received = true;
        last_pose_time = node->now();

        // Spin up the tracking loop on the first valid pose.
        if (!tracking_active.load())
        {
          if (move_to_pose_thread && move_to_pose_thread->joinable())
            move_to_pose_thread->join();

          tracking_active = true;
          tracker.resetTargetPose();
          move_to_pose_thread = std::make_shared<std::thread>([&]() {
            RCLCPP_INFO(LOGGER, "Pose tracking started");
            while (tracking_active.load() && rclcpp::ok())
            {
              moveit_servo::PoseTrackingStatusCode status =
                  tracker.moveToPose(lin_tol, rot_tolerance, target_pose_timeout);
              if (status != moveit_servo::PoseTrackingStatusCode::SUCCESS)
              {
                RCLCPP_WARN_STREAM_THROTTLE(LOGGER, *node->get_clock(), 2000,
                                            "Pose tracking status: "
                                                << moveit_servo::POSE_TRACKING_STATUS_CODE_MAP.at(status));
              }
            }
            RCLCPP_INFO(LOGGER, "Pose tracking stopped");
          });
        }
      });

  RCLCPP_INFO(LOGGER, "Ready. Listening for target poses on: %s", hand_pose_topic.c_str());
  RCLCPP_INFO(LOGGER, "Run the MediaPipe node and show your RIGHT hand to the camera to start tracking.");

  // ---- Idle watchdog: stop tracking when no poses arrive ------------------
  rclcpp::WallRate loop_rate(50);
  while (rclcpp::ok())
  {
    const bool active = pose_received && (node->now() - last_pose_time).seconds() < no_pose_timeout;
    if (!active && tracking_active.load())
    {
      tracking_active = false;
      tracker.stopMotion();
      RCLCPP_INFO(LOGGER, "No target poses received - stopping tracking");
    }
    else if (!active)
    {
      RCLCPP_INFO_THROTTLE(LOGGER, *node->get_clock(), 5000, "Waiting for target poses on %s ...",
                           hand_pose_topic.c_str());
    }
    loop_rate.sleep();
  }

  tracking_active = false;
  if (move_to_pose_thread && move_to_pose_thread->joinable())
    move_to_pose_thread->join();
  executor->cancel();
  executor_thread.join();
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}