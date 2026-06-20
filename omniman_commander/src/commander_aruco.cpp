#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <aruco_opencv_msgs/msg/aruco_detection.hpp>

#include "omniman_commander/arm_commander.hpp"

using omniman::ArmCommander;
using omniman::Offset;
using omniman::apply_offset;
using namespace std::chrono_literals;

// ---------- Mission config loaded from aruco_pick.yaml (no recompile to change) ----------

struct MissionConfig {
  int    marker_id              = 0;
  double detect_timeout         = 10.0;   // seconds to wait for a marker
  bool   use_marker_orientation = false;  // false = grasp with ready-orientation + offset
  Offset look, approach, grasp, place;
};

template<typename T>
static T get_param(rclcpp::Node::SharedPtr node, const std::string & name, T default_val)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter(name, default_val);
  }
  return node->get_parameter(name).get_value<T>();
}

static Offset load_offset(rclcpp::Node::SharedPtr node, const std::string & name)
{
  Offset o;
  o.dx     = get_param(node, name + ".dx",     0.0);
  o.dy     = get_param(node, name + ".dy",     0.0);
  o.dz     = get_param(node, name + ".dz",     0.0);
  o.droll  = get_param(node, name + ".droll",  0.0);
  o.dpitch = get_param(node, name + ".dpitch", 0.0);
  o.dyaw   = get_param(node, name + ".dyaw",   0.0);
  return o;
}

static MissionConfig load_mission(rclcpp::Node::SharedPtr node)
{
  MissionConfig cfg;
  cfg.marker_id              = get_param(node, "marker_id", 0);
  cfg.detect_timeout         = get_param(node, "detect_timeout", 10.0);
  cfg.use_marker_orientation = get_param(node, "use_marker_orientation", false);
  cfg.look     = load_offset(node, "look");
  cfg.approach = load_offset(node, "approach");
  cfg.grasp    = load_offset(node, "grasp");
  cfg.place    = load_offset(node, "place");
  return cfg;
}

// ---------- State machine ----------

enum class State {
  READY,
  OPEN_GRIPPER,
  LOOK,
  DETECT,
  APPROACH,
  GRASP,
  CLOSE_GRIPPER,
  LIFT,
  PLACE,
  RELEASE,
  RETURN_READY,
  DONE,
  ERROR
};

static const char * state_name(State s)
{
  switch (s) {
    case State::READY:         return "READY";
    case State::OPEN_GRIPPER:  return "OPEN_GRIPPER";
    case State::LOOK:          return "LOOK";
    case State::DETECT:        return "DETECT";
    case State::APPROACH:      return "APPROACH";
    case State::GRASP:         return "GRASP";
    case State::CLOSE_GRIPPER: return "CLOSE_GRIPPER";
    case State::LIFT:          return "LIFT";
    case State::PLACE:         return "PLACE";
    case State::RELEASE:       return "RELEASE";
    case State::RETURN_READY:  return "RETURN_READY";
    case State::DONE:          return "DONE";
    case State::ERROR:         return "ERROR";
    default:                   return "UNKNOWN";
  }
}

// ---------- ArUco detection holder ----------
// The aruco_tracker node publishes /aruco_detections. We cache the newest message
// (under a mutex) from the subscription callback; the DETECT state reads it.

class DetectionCache
{
public:
  void update(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    latest_ = *msg;
    have_   = true;
  }

  // Returns true and fills `pose_out` + `frame_out` + `stamp_out` if a marker with
  // `marker_id` is present in the most recent detection message.
  bool find_marker(int marker_id,
                   geometry_msgs::msg::Pose & pose_out,
                   std::string & frame_out,
                   rclcpp::Time & stamp_out)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!have_) return false;
    for (const auto & m : latest_.markers) {
      if (m.marker_id == marker_id) {
        pose_out  = m.pose;
        frame_out = latest_.header.frame_id;
        stamp_out = latest_.header.stamp;
        return true;
      }
    }
    return false;
  }

private:
  std::mutex mtx_;
  aruco_opencv_msgs::msg::ArucoDetection latest_;
  bool have_ = false;
};

// ---------- Main ----------

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "commander_aruco",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = rclcpp::get_logger("commander_aruco");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  auto cfg = load_mission(node);

  auto planner_pipeline = get_param(node, "planner_pipeline",
                                    std::string("pilz_industrial_motion_planner"));
  auto planner_id       = get_param(node, "planner_id", std::string("PTP"));
  auto velocity_scale     = get_param(node, "velocity_scale",     0.3);
  auto acceleration_scale = get_param(node, "acceleration_scale", 0.3);

  // The frame that Cartesian goals (setPoseTarget) are expressed in — the MoveGroup
  // planning frame. On this robot that is the URDF root "base_footprint". Detected
  // marker poses are transformed into this frame before being used as goals.
  auto planning_frame = get_param(node, "planning_frame", std::string("base_footprint"));

  // TF: marker poses arrive in the camera optical frame; we transform them into the
  // MoveGroup planning frame so they can be used as Cartesian goals.
  auto tf_buffer   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // ArUco detections subscription.
  auto cache = std::make_shared<DetectionCache>();
  auto sub = node->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
    "/aruco_detections", rclcpp::SensorDataQoS(),
    [cache](const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg) {
      cache->update(msg);
    });

  ArmCommander commander(node);
  commander.set_planner(planner_pipeline, planner_id);
  commander.set_scaling(velocity_scale, acceleration_scale);
  commander.add_floor();
  RCLCPP_INFO(logger, "ArUco pick mission — target marker id: %d", cfg.marker_id);
  RCLCPP_INFO(logger, "Transforming marker poses into planning frame: %s",
    planning_frame.c_str());

  State state = State::READY;
  bool  ok    = true;

  geometry_msgs::msg::Pose ready_pose;   // captured at READY; supplies the PLACE reference
  geometry_msgs::msg::Pose look_pose;    // captured at LOOK; supplies the grasp orientation (tilted down)
  geometry_msgs::msg::Pose object_pose;  // marker position in the planning frame

  while (rclcpp::ok() && state != State::DONE && state != State::ERROR) {
    RCLCPP_INFO(logger, "====== State: %s ======", state_name(state));

    switch (state) {
      case State::READY:
        ok = commander.move_named("ready");
        if (ok) {
          ready_pose = commander.current_pose();
          RCLCPP_INFO(logger, "Ready pose (observation): pos=(%.3f, %.3f, %.3f)",
            ready_pose.position.x, ready_pose.position.y, ready_pose.position.z);
        }
        state = ok ? State::OPEN_GRIPPER : State::ERROR;
        break;

      case State::OPEN_GRIPPER:
        ok    = commander.move_gripper("open");
        state = ok ? State::LOOK : State::ERROR;
        break;

      case State::LOOK:
        ok = commander.move_to_pose(apply_offset(ready_pose, cfg.look), "look");
        if (ok) {
          // Capture the tilted-down orientation here — this is the natural grasp
          // orientation (gripper angled at the object), used by APPROACH/GRASP below.
          look_pose = commander.current_pose();
        }
        state = ok ? State::DETECT : State::ERROR;
        break;

      case State::DETECT: {
        // Poll the cache until the target marker appears or detect_timeout elapses.
        ok = false;
        auto deadline = node->now() + rclcpp::Duration::from_seconds(cfg.detect_timeout);
        geometry_msgs::msg::Pose marker_pose;
        std::string marker_frame;
        rclcpp::Time  marker_stamp;

        while (rclcpp::ok() && node->now() < deadline) {
          if (cache->find_marker(cfg.marker_id, marker_pose, marker_frame, marker_stamp)) {
            // Transform the marker pose from the camera frame into the planning frame.
            geometry_msgs::msg::PoseStamped in, out;
            in.header.frame_id = marker_frame;
            in.header.stamp    = marker_stamp;
            in.pose            = marker_pose;
            try {
              out = tf_buffer->transform(in, planning_frame, 200ms);
            } catch (const tf2::TransformException & ex) {
              RCLCPP_WARN(logger, "  TF %s -> %s failed: %s — retrying",
                marker_frame.c_str(), planning_frame.c_str(), ex.what());
              std::this_thread::sleep_for(100ms);
              continue;
            }
            object_pose = out.pose;
            RCLCPP_INFO(logger,
              "Marker %d found. Object in %s: pos=(%.3f, %.3f, %.3f)",
              cfg.marker_id, planning_frame.c_str(),
              object_pose.position.x, object_pose.position.y, object_pose.position.z);
            ok = true;
            break;
          }
          std::this_thread::sleep_for(100ms);
        }

        if (!ok) {
          RCLCPP_ERROR(logger, "  Marker %d not detected within %.1fs",
            cfg.marker_id, cfg.detect_timeout);
        }
        state = ok ? State::APPROACH : State::ERROR;
        break;
      }

      case State::APPROACH: {
        // Reference = detected object position + the orientation we want to grasp with.
        // Default: use the LOOK orientation (gripper tilted down at the object); set
        // use_marker_orientation:=true to grasp using the marker's own orientation.
        geometry_msgs::msg::Pose ref;
        ref.position    = object_pose.position;
        ref.orientation = cfg.use_marker_orientation ? object_pose.orientation
                                                      : look_pose.orientation;
        ok = commander.move_to_pose(apply_offset(ref, cfg.approach), "approach");
        state = ok ? State::GRASP : State::ERROR;
        break;
      }

      case State::GRASP: {
        geometry_msgs::msg::Pose ref;
        ref.position    = object_pose.position;
        ref.orientation = cfg.use_marker_orientation ? object_pose.orientation
                                                      : look_pose.orientation;
        ok = commander.move_to_pose(apply_offset(ref, cfg.grasp), "grasp");
        state = ok ? State::CLOSE_GRIPPER : State::ERROR;
        break;
      }

      case State::CLOSE_GRIPPER:
        ok    = commander.move_gripper("close");
        state = ok ? State::LIFT : State::ERROR;
        break;

      case State::LIFT:
        ok    = commander.move_named("ready");
        state = ok ? State::PLACE : State::ERROR;
        break;

      case State::PLACE:
        // Place is a fixed drop location: offset from the ready pose.
        ok = commander.move_to_pose(apply_offset(ready_pose, cfg.place), "place");
        state = ok ? State::RELEASE : State::ERROR;
        break;

      case State::RELEASE:
        ok    = commander.move_gripper("open");
        state = ok ? State::RETURN_READY : State::ERROR;
        break;

      case State::RETURN_READY:
        ok    = commander.move_named("ready");
        state = ok ? State::DONE : State::ERROR;
        break;

      default:
        state = State::ERROR;
        break;
    }
  }

  if (state == State::DONE) {
    RCLCPP_INFO(logger, "ArUco pick and place complete!");
  } else {
    RCLCPP_ERROR(logger, "ArUco pick and place FAILED. Check logs above for the failing state.");
  }

  rclcpp::shutdown();
  spinner.join();
  return (state == State::DONE) ? 0 : 1;
}
