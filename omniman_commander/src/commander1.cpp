#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

#include "omniman_commander/arm_commander.hpp"

using omniman::apply_offset;
using omniman::ArmCommander;
using omniman::Offset;

// ---------- Mission config loaded from poses.yaml (no recompile to change poses) ----------

struct MissionConfig {
  Offset pre_grasp, grasp, lift, place;
};

// automatically_declare_parameters_from_overrides pre-declares params from YAML,
// so we check before declaring to avoid ParameterAlreadyDeclaredException.
template <typename T>
static T get_param(rclcpp::Node::SharedPtr node, const std::string& name, T default_val) {
  if (!node->has_parameter(name)) {
    node->declare_parameter(name, default_val);
  }
  return node->get_parameter(name).get_value<T>();
}

static Offset load_offset(rclcpp::Node::SharedPtr node, const std::string& name) {
  Offset o;
  o.dx = get_param(node, name + ".dx", 0.0);
  o.dy = get_param(node, name + ".dy", 0.0);
  o.dz = get_param(node, name + ".dz", 0.0);
  o.droll = get_param(node, name + ".droll", 0.0);
  o.dpitch = get_param(node, name + ".dpitch", 0.0);
  o.dyaw = get_param(node, name + ".dyaw", 0.0);
  return o;
}

static MissionConfig load_mission(rclcpp::Node::SharedPtr node) {
  MissionConfig cfg;
  cfg.pre_grasp = load_offset(node, "pre_grasp");
  cfg.grasp = load_offset(node, "grasp");
  cfg.lift = load_offset(node, "lift");
  cfg.place = load_offset(node, "place");
  return cfg;
}

// ---------- State machine ----------

enum class State {
  READY,
  OPEN_GRIPPER,
  PRE_GRASP,
  GRASP,
  CLOSE_GRIPPER,
  LIFT,
  PLACE,
  RELEASE,
  RETURN_READY,
  DONE,
  ERROR
};

static const char* state_name(State s) {
  switch (s) {
    case State::READY:
      return "READY";
    case State::OPEN_GRIPPER:
      return "OPEN_GRIPPER";
    case State::PRE_GRASP:
      return "PRE_GRASP";
    case State::GRASP:
      return "GRASP";
    case State::CLOSE_GRIPPER:
      return "CLOSE_GRIPPER";
    case State::LIFT:
      return "LIFT";
    case State::PLACE:
      return "PLACE";
    case State::RELEASE:
      return "RELEASE";
    case State::RETURN_READY:
      return "RETURN_READY";
    case State::DONE:
      return "DONE";
    case State::ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}

// ---------- Main ----------

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "commander1", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = rclcpp::get_logger("commander1");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  auto cfg = load_mission(node);

  // Planner selection comes from poses.yaml (defaults to Pilz PTP if not set).
  auto planner_pipeline =
      get_param(node, "planner_pipeline", std::string("pilz_industrial_motion_planner"));
  auto planner_id = get_param(node, "planner_id", std::string("PTP"));

  auto velocity_scale = get_param(node, "velocity_scale", 0.3);
  auto acceleration_scale = get_param(node, "acceleration_scale", 0.3);

  ArmCommander commander(node);
  commander.set_planner(planner_pipeline, planner_id);
  commander.set_scaling(velocity_scale, acceleration_scale);
  RCLCPP_INFO(logger, "Mission planner from yaml -> pipeline: %s, planner: %s",
              planner_pipeline.c_str(), planner_id.c_str());
  commander.add_floor();

  State state = State::READY;
  bool ok = true;

  // ref_pose tracks the previous step's achieved pose; each offset is applied on top of it.
  geometry_msgs::msg::Pose ref_pose;

  while (rclcpp::ok() && state != State::DONE && state != State::ERROR) {
    RCLCPP_INFO(logger, "====== State: %s ======", state_name(state));

    switch (state) {
      case State::READY:
        ok = commander.move_named("ready");
        if (ok) {
          ref_pose = commander.current_pose();
          RCLCPP_INFO(
              logger, "Captured ready pose: pos=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)",
              ref_pose.position.x, ref_pose.position.y, ref_pose.position.z, ref_pose.orientation.x,
              ref_pose.orientation.y, ref_pose.orientation.z, ref_pose.orientation.w);
        }
        state = ok ? State::OPEN_GRIPPER : State::ERROR;
        break;

      case State::OPEN_GRIPPER:
        ok = commander.move_gripper("open");
        state = ok ? State::PRE_GRASP : State::ERROR;
        break;

      case State::PRE_GRASP:
        ok = commander.move_to_pose(apply_offset(ref_pose, cfg.pre_grasp), "pre_grasp");
        if (ok) ref_pose = commander.current_pose();
        state = ok ? State::GRASP : State::ERROR;
        break;

      case State::GRASP:
        ok = commander.move_to_pose(apply_offset(ref_pose, cfg.grasp), "grasp");
        if (ok) ref_pose = commander.current_pose();
        state = ok ? State::CLOSE_GRIPPER : State::ERROR;
        break;

      case State::CLOSE_GRIPPER:
        ok = commander.move_gripper("close");
        state = ok ? State::LIFT : State::ERROR;
        break;

      case State::LIFT:
        ok = commander.move_to_pose(apply_offset(ref_pose, cfg.lift), "lift");
        if (ok) ref_pose = commander.current_pose();
        state = ok ? State::PLACE : State::ERROR;
        break;

      case State::PLACE:
        ok = commander.move_to_pose(apply_offset(ref_pose, cfg.place), "place");
        if (ok) ref_pose = commander.current_pose();
        state = ok ? State::RELEASE : State::ERROR;
        break;

      case State::RELEASE:
        ok = commander.move_gripper("open");
        state = ok ? State::RETURN_READY : State::ERROR;
        break;

      case State::RETURN_READY:
        ok = commander.move_named("ready");
        state = ok ? State::DONE : State::ERROR;
        break;

      default:
        state = State::ERROR;
        break;
    }
  }

  if (state == State::DONE) {
    RCLCPP_INFO(logger, "Pick and place complete!");
  } else {
    RCLCPP_ERROR(logger, "Pick and place FAILED. Check logs above for the failing state.");
  }

  rclcpp::shutdown();
  spinner.join();
  return (state == State::DONE) ? 0 : 1;
}
