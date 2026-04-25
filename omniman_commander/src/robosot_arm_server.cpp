// Robosot arm command server.
//
// Receives high-level commands on /robosot/arm_cmd (std_msgs/String) and
// publishes results on /robosot/arm_result (std_msgs/String).
//
// Commands:
//   named:POSE          → arm to SRDF named pose (e.g. "named:left")
//   gripper:open        → open gripper
//   gripper:close       → close gripper
//   pick:LETTER         → at search pose: align→open→descend→offset→close→lift
//   place:POSE          → arm to POSE, open, retract, lift  (assumes pre-staged)
//   scan:F,I,R,A        → at search pose, return first letter YOLO sees
//
// Results:
//   ok                  → success
//   fail                → failure
//   ok:LETTER           → for scan, the letter that was found
//   none                → for scan, no letter found
//
// The Python mission owns navigation + line-PID and orchestrates the full
// pick-and-place flow by sending these commands one at a time.

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <algorithm>
#include <sstream>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
namespace rvt = rviz_visual_tools;
using moveit::planning_interface::MoveGroupInterface;

// =====================================================================
// YOLO + arm motion tuning (same defaults as fira_mission_yolo)
// =====================================================================
static constexpr double STEP_XY            = 0.01;
static constexpr double P_GAIN             = 1e-4;
static constexpr double STEP_Z             = -0.14;
static constexpr double GO_UP_Z            = 0.1;
static constexpr double RETRACT_Y          = 0.02;
static constexpr auto   POST_ACTION_DELAY  = 500ms;
static constexpr double PIXEL_THRESHOLD    = 15.0;
static constexpr int    MAX_ALIGN_ITERS    = 30;
static constexpr auto   ERROR_WAIT_TIMEOUT = 3s;
static constexpr auto   DETECTION_TIMEOUT  = 5s;
static const std::string ERROR_TOPIC_PREFIX = "/yolo_error_";

static constexpr double CAMERA_OFFSET_X    = 0.05;
static constexpr double CAMERA_OFFSET_Y    = 0.0;
static constexpr double SIGN_ERR_X_TO_DY   = -1.0;
static constexpr double SIGN_ERR_Y_TO_DX   = -1.0;

// =====================================================================
// Commander1 — arm + gripper helpers (same as fira_mission_yolo)
// =====================================================================
struct CommanderParams {
  double arm_max_velocity_scaling_factor = 0.15;
  double arm_max_acceleration_scaling_factor = 0.15;
  std::string end_effector_link = "ee_link";
};

class Commander1 {
public:
  Commander1(std::shared_ptr<rclcpp::Logger> logger,
             MoveGroupInterface& arm,
             MoveGroupInterface& gripper,
             moveit_visual_tools::MoveItVisualTools& vt,
             CommanderParams& params)
    : logger_(logger), arm_(&arm), gripper_(&gripper), vt_(&vt), params_(&params)
  {
    jmg_ = arm_->getRobotModel()->getJointModelGroup("arm");
  }

  void clear_target_n_constraints() {
    arm_->clearPoseTargets();
    arm_->clearPathConstraints();
  }

  bool move_ptp(const geometry_msgs::msg::Pose& target, const std::string& label,
                const moveit::core::LinkModel* ee_link) {
    RCLCPP_INFO(*logger_, "Move `%s`: pos=(%.4f, %.4f, %.4f)", label.c_str(),
                target.position.x, target.position.y, target.position.z);
    clear_target_n_constraints();
    arm_->setPlannerId("PTP");
    arm_->setMaxVelocityScalingFactor(params_->arm_max_velocity_scaling_factor);
    arm_->setMaxAccelerationScalingFactor(params_->arm_max_acceleration_scaling_factor);
    arm_->setStartState(*arm_->getCurrentState());
    arm_->setPoseTarget(target, params_->end_effector_link);

    MoveGroupInterface::Plan plan;
    if (arm_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(*logger_, "Plan failed for `%s`", label.c_str());
      return false;
    }
    vt_->publishTrajectoryLine(plan.trajectory_, ee_link, jmg_);
    vt_->trigger();
    auto exec = arm_->execute(plan);
    bool ok = exec == moveit::core::MoveItErrorCode::SUCCESS;
    if (!ok) RCLCPP_ERROR(*logger_, "Execute failed for `%s`: %d", label.c_str(),
                          static_cast<int>(exec.val));
    return ok;
  }

  bool move_arm_to_named(const std::string& name) {
    RCLCPP_INFO(*logger_, "Arm → `%s`", name.c_str());
    clear_target_n_constraints();
    arm_->setStartStateToCurrentState();
    arm_->setPlannerId("PTP");
    arm_->setMaxVelocityScalingFactor(params_->arm_max_velocity_scaling_factor);
    arm_->setMaxAccelerationScalingFactor(params_->arm_max_acceleration_scaling_factor);
    arm_->setNamedTarget(name);
    MoveGroupInterface::Plan plan;
    if (arm_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(*logger_, "Arm plan failed for `%s`", name.c_str());
      return false;
    }
    return arm_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool move_gripper(const std::string& name) {
    constexpr int kMax = 3;
    for (int i = 1; i <= kMax && rclcpp::ok(); ++i) {
      RCLCPP_INFO(*logger_, "Gripper → `%s` (try %d/%d)", name.c_str(), i, kMax);
      gripper_->clearPoseTargets();
      gripper_->setStartStateToCurrentState();
      gripper_->setNamedTarget(name);
      MoveGroupInterface::Plan plan;
      if (gripper_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        std::this_thread::sleep_for(200ms);
        continue;
      }
      if (gripper_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) return true;
      std::this_thread::sleep_for(200ms);
    }
    return false;
  }

  MoveGroupInterface& arm_move_group() { return *arm_; }

private:
  std::shared_ptr<rclcpp::Logger> logger_;
  MoveGroupInterface* arm_;
  MoveGroupInterface* gripper_;
  moveit_visual_tools::MoveItVisualTools* vt_;
  CommanderParams* params_;
  const moveit::core::JointModelGroup* jmg_;
};

// =====================================================================
// ArmServer — listens on /robosot/arm_cmd, dispatches arm operations
// =====================================================================
class ArmServer {
public:
  ArmServer(rclcpp::Node::SharedPtr node,
            std::shared_ptr<rclcpp::Logger> logger,
            Commander1& commander,
            const moveit::core::LinkModel* ee_link)
    : node_(node), logger_(logger), commander_(&commander), ee_link_(ee_link)
  {
    cmd_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/robosot/arm_cmd", 10,
      [this](std_msgs::msg::String::SharedPtr msg) {
        // Dispatch on a worker thread so MoveIt blocking calls don't stall the executor.
        std::thread([this, payload = msg->data]() { handle_cmd(payload); }).detach();
      });
    result_pub_ = node_->create_publisher<std_msgs::msg::String>("/robosot/arm_result", 10);
    RCLCPP_INFO(*logger_, "ArmServer ready. Listening on /robosot/arm_cmd");
  }

private:
  void publish_result(const std::string& result) {
    std_msgs::msg::String msg;
    msg.data = result;
    result_pub_->publish(msg);
    RCLCPP_INFO(*logger_, "Reply: `%s`", result.c_str());
  }

  void handle_cmd(const std::string& cmd) {
    RCLCPP_INFO(*logger_, "Command: `%s`", cmd.c_str());

    auto colon = cmd.find(':');
    std::string verb = (colon == std::string::npos) ? cmd : cmd.substr(0, colon);
    std::string arg  = (colon == std::string::npos) ? ""  : cmd.substr(colon + 1);

    if (verb == "named") {
      publish_result(commander_->move_arm_to_named(arg) ? "ok" : "fail");
    } else if (verb == "gripper") {
      publish_result(commander_->move_gripper(arg) ? "ok" : "fail");
    } else if (verb == "pick") {
      publish_result(do_pick(arg) ? "ok" : "fail");
    } else if (verb == "place") {
      publish_result(do_place(arg) ? "ok" : "fail");
    } else if (verb == "scan") {
      auto found = do_scan(arg);
      publish_result(found.empty() ? "none" : ("ok:" + found));
    } else {
      RCLCPP_WARN(*logger_, "Unknown command verb: `%s`", verb.c_str());
      publish_result("fail");
    }
  }

  // ----- pick: assume already at search pose; align + descend + offset + close + lift -----
  bool do_pick(const std::string& letter) {
    const std::string topic = ERROR_TOPIC_PREFIX + to_lower(letter);
    subscribe_error(topic);
    if (!wait_fresh_error(DETECTION_TIMEOUT)) {
      RCLCPP_WARN(*logger_, "[%s] No detection", letter.c_str());
      return false;
    }
    if (!align()) return false;
    if (!commander_->move_gripper("open")) return false;
    if (!descend()) return false;
    if (!apply_camera_offset()) return false;
    if (!commander_->move_gripper("close")) return false;
    std::this_thread::sleep_for(POST_ACTION_DELAY);
    if (!go_up()) return false;
    return true;
  }

  // ----- place: arm → POSE, open, retract, lift -----
  bool do_place(const std::string& pose_name) {
    if (!commander_->move_arm_to_named(pose_name)) return false;
    if (!commander_->move_gripper("open")) return false;
    std::this_thread::sleep_for(POST_ACTION_DELAY);
    if (!retract()) return false;
    std::this_thread::sleep_for(POST_ACTION_DELAY);
    if (!go_up()) return false;
    return true;
  }

  // ----- scan: try each letter, return first found -----
  std::string do_scan(const std::string& letters_csv) {
    std::vector<std::string> letters;
    std::stringstream ss(letters_csv);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
      if (!tok.empty()) letters.push_back(tok);
    }
    for (const auto& letter : letters) {
      const std::string topic = ERROR_TOPIC_PREFIX + to_lower(letter);
      subscribe_error(topic);
      RCLCPP_INFO(*logger_, "Scanning `%s` on %s...", letter.c_str(), topic.c_str());
      if (wait_fresh_error(DETECTION_TIMEOUT)) {
        RCLCPP_INFO(*logger_, "Scan: detected `%s`", letter.c_str());
        return letter;
      }
    }
    return "";
  }

  // ----- YOLO alignment helpers (same as fira_mission_yolo) -----
  void subscribe_error(const std::string& topic) {
    error_sub_.reset();
    {
      std::lock_guard<std::mutex> lock(error_mu_);
      latest_err_x_ = 0.0;
      latest_err_y_ = 0.0;
      error_fresh_ = false;
    }
    error_sub_ = node_->create_subscription<geometry_msgs::msg::Point>(
      topic, 10,
      [this](geometry_msgs::msg::Point::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(error_mu_);
        latest_err_x_ = msg->x;
        latest_err_y_ = msg->y;
        error_fresh_ = true;
        error_cv_.notify_all();
      });
  }

  bool wait_fresh_error(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(error_mu_);
    error_fresh_ = false;
    return error_cv_.wait_for(lock, timeout, [this] { return error_fresh_ || !rclcpp::ok(); })
           && error_fresh_;
  }

  bool align() {
    for (int i = 0; i < MAX_ALIGN_ITERS && rclcpp::ok(); ++i) {
      double ex, ey;
      {
        std::lock_guard<std::mutex> lock(error_mu_);
        ex = latest_err_x_;
        ey = latest_err_y_;
      }
      RCLCPP_INFO(*logger_, "Align iter %d: err=(%+.1f, %+.1f)", i, ex, ey);
      if (std::abs(ex) < PIXEL_THRESHOLD && std::abs(ey) < PIXEL_THRESHOLD) return true;

      double dx = 0.0, dy = 0.0;
      if (std::abs(ex) >= PIXEL_THRESHOLD) {
        double mag = std::min(P_GAIN * std::abs(ex), STEP_XY);
        dy = SIGN_ERR_X_TO_DY * ((ex > 0) ? mag : -mag);
      }
      if (std::abs(ey) >= PIXEL_THRESHOLD) {
        double mag = std::min(P_GAIN * std::abs(ey), STEP_XY);
        dx = SIGN_ERR_Y_TO_DX * ((ey > 0) ? mag : -mag);
      }
      auto& arm = commander_->arm_move_group();
      auto cp = arm.getCurrentPose("ee_link").pose;
      geometry_msgs::msg::Pose tp = cp;
      tp.position.x += dx;
      tp.position.y += dy;

      commander_->clear_target_n_constraints();
      arm.setStartStateToCurrentState();
      if (!commander_->move_ptp(tp, "AlignStep", ee_link_)) return false;
      if (!wait_fresh_error(ERROR_WAIT_TIMEOUT)) return false;
    }
    return false;
  }

  bool apply_camera_offset() {
    auto& arm = commander_->arm_move_group();
    auto start = arm.getCurrentPose("ee_link").pose;
    const double fx = start.position.x + CAMERA_OFFSET_X;
    const double fy = start.position.y + CAMERA_OFFSET_Y;
    geometry_msgs::msg::Pose sub = start;
    while (rclcpp::ok()) {
      double rx = fx - sub.position.x, ry = fy - sub.position.y;
      if (std::abs(rx) < 1e-6 && std::abs(ry) < 1e-6) break;
      sub.position.x += std::clamp(rx, -STEP_XY, STEP_XY);
      sub.position.y += std::clamp(ry, -STEP_XY, STEP_XY);
      bool ok = false;
      for (int a = 1; a <= 3 && rclcpp::ok(); ++a) {
        commander_->clear_target_n_constraints();
        arm.setStartStateToCurrentState();
        if (commander_->move_ptp(sub, "CameraOffset", ee_link_)) { ok = true; break; }
        std::this_thread::sleep_for(250ms);
      }
      if (!ok) return false;
      std::this_thread::sleep_for(150ms);
    }
    return true;
  }

  bool descend() { return move_relative(0, 0, STEP_Z, "Descend"); }
  bool go_up()   { return move_relative(0, 0, GO_UP_Z, "GoUp"); }
  bool retract() { return move_relative(0, RETRACT_Y, 0, "Retract"); }

  bool move_relative(double dx, double dy, double dz, const std::string& label) {
    auto& arm = commander_->arm_move_group();
    auto cp = arm.getCurrentPose("ee_link").pose;
    geometry_msgs::msg::Pose tp = cp;
    tp.position.x += dx;
    tp.position.y += dy;
    tp.position.z += dz;
    commander_->clear_target_n_constraints();
    arm.setStartStateToCurrentState();
    return commander_->move_ptp(tp, label, ee_link_);
  }

  static std::string to_lower(const std::string& s) {
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::Logger> logger_;
  Commander1* commander_;
  const moveit::core::LinkModel* ee_link_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr error_sub_;
  std::mutex error_mu_;
  std::condition_variable error_cv_;
  double latest_err_x_ = 0.0;
  double latest_err_y_ = 0.0;
  bool error_fresh_ = false;
};

// =====================================================================
// main
// =====================================================================
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "robosot_arm_server",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = rclcpp::get_logger("robosot_arm_server");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  auto arm_mg = MoveGroupInterface(node, "arm");
  auto grip_mg = MoveGroupInterface(node, "gripper");
  arm_mg.setPlanningPipelineId("pilz_industrial_motion_planner");
  arm_mg.setPlannerId("PTP");

  auto vt = moveit_visual_tools::MoveItVisualTools{
    node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, arm_mg.getRobotModel()};
  vt.deleteAllMarkers();
  vt.loadRemoteControl();

  // Floor collision object.
  {
    moveit::planning_interface::PlanningSceneInterface psi;
    moveit_msgs::msg::CollisionObject floor;
    floor.header.frame_id = arm_mg.getPlanningFrame();
    floor.id = "floor";
    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {2.0, 2.0, 0.01};
    geometry_msgs::msg::Pose fp;
    fp.orientation.w = 1.0;
    fp.position.z = -0.01;
    floor.primitives.push_back(box);
    floor.primitive_poses.push_back(fp);
    floor.operation = floor.ADD;
    psi.applyCollisionObject(floor);
  }

  CommanderParams params;
  auto logger_ptr = std::make_shared<rclcpp::Logger>(logger);
  Commander1 commander(logger_ptr, arm_mg, grip_mg, vt, params);

  auto ee_link = arm_mg.getRobotModel()->getLinkModel("ee_link");
  ArmServer server(node, logger_ptr, commander, ee_link);

  RCLCPP_INFO(logger, "robosot_arm_server up. Spinning.");
  // Keep main thread alive until shutdown.
  while (rclcpp::ok()) std::this_thread::sleep_for(100ms);

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
