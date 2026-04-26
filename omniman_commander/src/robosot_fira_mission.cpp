// Robosot FIRA pick-and-place mission with mecanum navigation.
//
// Combines:
//   - MoveIt arm + gripper control (Commander1, reused from fira_mission_yolo)
//   - YOLO-based visual alignment for picking (same error topics)
//   - Odometry-based mecanum navigation via /cmd_vel
//
// Mission flow:
//   1. Open gripper, arm → "left" pose
//   2. Strafe right 0.5 m  (arm in "left" so camera faces lines for PID)
//   3. Drive backward 0.3 m → arrive at HOME
//   4. For each goods area (yellow / blue / green, same spokes as traffic_mission):
//      a. Navigate home → goods area
//      b. Arm → search pose, YOLO align, pick letter
//      c. Arm → transit pose, navigate goods → home
//      d. Arm → place pose (F1/I1/R1/A1), open gripper
//      e. Repeat until all 4 letters placed
//
// Tuning:  all distances, speeds, poses, and sign conventions are in the
//          constants block at the top.  Edit there, rebuild, run.

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;
namespace rvt = rviz_visual_tools;
using moveit::planning_interface::MoveGroupInterface;

// =====================================================================
// YOLO pick tuning (same defaults as fira_mission_yolo)
// =====================================================================
static constexpr double STEP_XY           = 0.01;
static constexpr double P_GAIN            = 1e-4;
static constexpr double STEP_Z            = -0.14;
static constexpr double GO_UP_Z           = 0.1;
static constexpr double RETRACT_Y         = 0.02;
static constexpr auto   POST_ACTION_DELAY = 500ms;
static constexpr double PIXEL_THRESHOLD   = 15.0;
static constexpr int    MAX_ALIGN_ITERS   = 30;
static constexpr auto   ERROR_WAIT_TIMEOUT = 3s;
static constexpr auto   DETECTION_TIMEOUT  = 5s;
static const std::string ERROR_TOPIC_PREFIX = "/yolo_error_";

static constexpr double CAMERA_OFFSET_X   = 0.05;
static constexpr double CAMERA_OFFSET_Y   = 0.0;
static constexpr double SIGN_ERR_X_TO_DY  = -1.0;
static constexpr double SIGN_ERR_Y_TO_DX  = -1.0;

// =====================================================================
// Navigation tuning
// =====================================================================
static const std::string CMD_VEL_TOPIC = "/cmd_vel";
static const std::string ODOM_TOPIC    = "/mecanum_drive_controller/odometry";

static constexpr double NAV_LINEAR_SPEED   = 0.10;  // m/s for straight / strafe
static constexpr double NAV_ANGULAR_SPEED  = 0.3;   // rad/s for turns
static constexpr double NAV_DISTANCE_TOL   = 0.02;  // m  — stop when within this of target
static constexpr double NAV_ANGLE_TOL      = 0.03;  // rad
static constexpr double NAV_RATE_HZ        = 20.0;

// =====================================================================
// Mission geometry
// =====================================================================
// Initial movement from start to HOME (robot faces opposite direction).
// Sequence: arm:north → backward → arm:left → strafe LEFT → backward.
static constexpr double INIT_PRE_BACKWARD_M = 0.5;
static constexpr double INIT_STRAFE_RIGHT_M = 0.8;   // strafe LEFT magnitude (sign applied at call site)
static constexpr double INIT_BACKWARD_M     = 0.3;

// Goods-area spokes (distances from HOME, same as traffic_mission).
// Each spoke: turn → drive → (pick) → reverse → turn back.
struct Spoke {
  std::string name;
  double turn_deg;          // yaw change before driving
  double drive_m;           // forward distance
  double approach_m;        // extra approach after finding letter
  double retreat_m;         // reverse after pick (negative = backward)
  bool   use_strafe;        // true = strafe instead of turn+drive
  double strafe_m;          // strafe distance (only if use_strafe)
};

// Tune these to match the real arena.
static const std::vector<Spoke> GOODS_SPOKES = {
    {"yellow",  +100.0, 0.40, 0.20, -0.40, false, 0.0},
    {"blue",      0.0,  0.40, 0.10, -0.40, true, -0.40},
    {"green",   -90.0,  1.00, 0.35, -0.35, false, 0.0},
};

// =====================================================================
// Arm pose names (SRDF)
// =====================================================================
static const std::string ARM_LEFT_POSE     = "left";
static const std::string ARM_RIGHT_POSE    = "right";
static const std::string SEARCH_POSE       = "north";
static const std::string TRANSIT_POSE      = "east";

static const std::vector<std::string> LETTERS       = {"F", "I", "R", "A"};
static const std::vector<std::string> PLACE_POSES   = {"F1", "I1", "R1", "A1"};

// =====================================================================
// Commander1 — arm + gripper helpers (identical to fira_mission_yolo)
// =====================================================================
struct CommanderParams {
  double arm_max_velocity_scaling_factor = 0.15;
  double arm_max_acceleration_scaling_factor = 0.15;
  std::string end_effector_link = "ee_link";
};

class Commander1 {
public:
  Commander1(
    std::shared_ptr<rclcpp::Logger> logger,
    MoveGroupInterface& arm_move_group,
    MoveGroupInterface& gripper_move_group,
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    CommanderParams& params)
    : logger_(logger),
      arm_move_group_(&arm_move_group),
      gripper_move_group_(&gripper_move_group),
      visual_tools_(&visual_tools),
      params_(&params)
  {
    text_pose_ = Eigen::Isometry3d::Identity();
    text_pose_.translation().z() = 1.0;
    joint_model_group_ = arm_move_group_->getRobotModel()->getJointModelGroup("arm");
  }

  void clear_target_n_constraints() {
    arm_move_group_->clearPoseTargets();
    arm_move_group_->clearPathConstraints();
  }

  bool move_ptp(const geometry_msgs::msg::Pose& target_pose, const std::string& text,
                const moveit::core::LinkModel* ee_link) {
    RCLCPP_INFO(*logger_, "Move to `%s`: pos=(%.4f, %.4f, %.4f)", text.c_str(),
      target_pose.position.x, target_pose.position.y, target_pose.position.z);

    clear_target_n_constraints();
    arm_move_group_->setPlannerId("PTP");
    arm_move_group_->setMaxVelocityScalingFactor(params_->arm_max_velocity_scaling_factor);
    arm_move_group_->setMaxAccelerationScalingFactor(params_->arm_max_acceleration_scaling_factor);

    auto current_state = arm_move_group_->getCurrentState();
    arm_move_group_->setStartState(*current_state);
    arm_move_group_->setPoseTarget(target_pose, params_->end_effector_link);

    MoveGroupInterface::Plan plan;
    auto plan_result = arm_move_group_->plan(plan);
    bool plan_ok = plan_result == moveit::core::MoveItErrorCode::SUCCESS;

    if (!plan_ok) {
      RCLCPP_WARN(*logger_, "Plan failed for `%s`", text.c_str());
      return false;
    }

    visual_tools_->publishTrajectoryLine(plan.trajectory_, ee_link, joint_model_group_);
    visual_tools_->trigger();

    auto exec_result = arm_move_group_->execute(plan);
    bool exec_ok = exec_result == moveit::core::MoveItErrorCode::SUCCESS;
    if (!exec_ok) {
      RCLCPP_ERROR(*logger_, "Execute failed for `%s`: %d", text.c_str(),
                   static_cast<int>(exec_result.val));
    }
    return exec_ok;
  }

  bool move_arm_to_named(const std::string& named_target) {
    RCLCPP_INFO(*logger_, "Arm -> `%s`", named_target.c_str());
    clear_target_n_constraints();
    arm_move_group_->setStartStateToCurrentState();
    arm_move_group_->setPlannerId("PTP");
    arm_move_group_->setMaxVelocityScalingFactor(params_->arm_max_velocity_scaling_factor);
    arm_move_group_->setMaxAccelerationScalingFactor(params_->arm_max_acceleration_scaling_factor);
    arm_move_group_->setNamedTarget(named_target);
    MoveGroupInterface::Plan aplan;
    auto plan_result = arm_move_group_->plan(aplan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(*logger_, "Arm plan failed for `%s`", named_target.c_str());
      return false;
    }
    auto exec_result = arm_move_group_->execute(aplan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(*logger_, "Arm execute failed for `%s`", named_target.c_str());
      return false;
    }
    return true;
  }

  bool move_gripper(const std::string& named_target) {
    constexpr int kMaxAttempts = 3;
    constexpr auto kRetryDelay = 200ms;
    for (int attempt = 1; attempt <= kMaxAttempts && rclcpp::ok(); ++attempt) {
      RCLCPP_INFO(*logger_, "Gripper -> `%s` (attempt %d/%d)",
                  named_target.c_str(), attempt, kMaxAttempts);
      gripper_move_group_->clearPoseTargets();
      gripper_move_group_->setStartStateToCurrentState();
      gripper_move_group_->setNamedTarget(named_target);
      MoveGroupInterface::Plan gplan;
      auto plan_result = gripper_move_group_->plan(gplan);
      if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(*logger_, "Gripper plan failed (attempt %d/%d)", attempt, kMaxAttempts);
        std::this_thread::sleep_for(kRetryDelay);
        continue;
      }
      auto exec_result = gripper_move_group_->execute(gplan);
      if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) return true;
      RCLCPP_WARN(*logger_, "Gripper exec failed (attempt %d/%d)", attempt, kMaxAttempts);
      std::this_thread::sleep_for(kRetryDelay);
    }
    RCLCPP_ERROR(*logger_, "Gripper `%s` gave up.", named_target.c_str());
    return false;
  }

  MoveGroupInterface& arm_move_group() { return *arm_move_group_; }

private:
  std::shared_ptr<rclcpp::Logger> logger_;
  MoveGroupInterface* arm_move_group_;
  MoveGroupInterface* gripper_move_group_;
  moveit_visual_tools::MoveItVisualTools* visual_tools_;
  CommanderParams* params_;
  Eigen::Isometry3d text_pose_;
  const moveit::core::JointModelGroup* joint_model_group_;
};

// =====================================================================
// NavHelper — odometry-based mecanum navigation
// =====================================================================
class NavHelper {
public:
  NavHelper(rclcpp::Node::SharedPtr node, std::shared_ptr<rclcpp::Logger> logger)
    : node_(node), logger_(logger)
  {
    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(CMD_VEL_TOPIC, 10);
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      ODOM_TOPIC, 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mu_);
        odom_x_ = msg->pose.pose.position.x;
        odom_y_ = msg->pose.pose.position.y;
        auto& q = msg->pose.pose.orientation;
        odom_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        odom_received_ = true;
      });
  }

  void publish_stop() {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = node_->get_clock()->now();
    cmd.header.frame_id = "base_link";
    cmd_pub_->publish(cmd);
  }

  bool wait_odom(std::chrono::milliseconds timeout = 5s) {
    auto start = std::chrono::steady_clock::now();
    while (!odom_received_ && rclcpp::ok()) {
      if (std::chrono::steady_clock::now() - start > timeout) return false;
      std::this_thread::sleep_for(50ms);
    }
    return true;
  }

  // Drive straight: +distance = forward, -distance = backward.
  bool drive_straight(double distance_m, double speed = NAV_LINEAR_SPEED) {
    RCLCPP_INFO(*logger_, "NAV: drive_straight %.2f m @ %.2f m/s", distance_m, speed);
    double sx, sy;
    snapshot(sx, sy);
    double target_dist = std::abs(distance_m);
    double vx = (distance_m >= 0) ? speed : -speed;

    rclcpp::Rate rate(NAV_RATE_HZ);
    while (rclcpp::ok()) {
      double cx, cy;
      snapshot(cx, cy);
      double traveled = std::hypot(cx - sx, cy - sy);
      if (traveled >= target_dist - NAV_DISTANCE_TOL) break;
      publish_twist(vx, 0.0, 0.0);
      rate.sleep();
    }
    publish_stop();
    RCLCPP_INFO(*logger_, "NAV: drive_straight done.");
    return true;
  }

  // Strafe: +distance = left (+y), -distance = right (-y).
  bool strafe(double distance_m, double speed = NAV_LINEAR_SPEED) {
    RCLCPP_INFO(*logger_, "NAV: strafe %.2f m @ %.2f m/s", distance_m, speed);
    double sx, sy;
    snapshot(sx, sy);
    double target_dist = std::abs(distance_m);
    double vy = (distance_m >= 0) ? speed : -speed;

    rclcpp::Rate rate(NAV_RATE_HZ);
    while (rclcpp::ok()) {
      double cx, cy;
      snapshot(cx, cy);
      double traveled = std::hypot(cx - sx, cy - sy);
      if (traveled >= target_dist - NAV_DISTANCE_TOL) break;
      publish_twist(0.0, vy, 0.0);
      rate.sleep();
    }
    publish_stop();
    RCLCPP_INFO(*logger_, "NAV: strafe done.");
    return true;
  }

  // Turn: +angle = CCW (left), -angle = CW (right). In radians.
  bool turn(double angle_rad, double speed = NAV_ANGULAR_SPEED) {
    RCLCPP_INFO(*logger_, "NAV: turn %.1f deg @ %.2f rad/s",
                angle_rad * 180.0 / M_PI, speed);
    double start_yaw = get_yaw();
    double target_yaw = wrap_angle(start_yaw + angle_rad);

    rclcpp::Rate rate(NAV_RATE_HZ);
    while (rclcpp::ok()) {
      double err = wrap_angle(target_yaw - get_yaw());
      if (std::abs(err) < NAV_ANGLE_TOL) break;
      double wz = std::clamp(2.0 * err, -speed, speed);
      publish_twist(0.0, 0.0, wz);
      rate.sleep();
    }
    publish_stop();
    RCLCPP_INFO(*logger_, "NAV: turn done. yaw=%.1f deg", get_yaw() * 180.0 / M_PI);
    return true;
  }

private:
  void publish_twist(double vx, double vy, double wz) {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = node_->get_clock()->now();
    cmd.header.frame_id = "base_link";
    cmd.twist.linear.x = vx;
    cmd.twist.linear.y = vy;
    cmd.twist.angular.z = wz;
    cmd_pub_->publish(cmd);
  }

  void snapshot(double& x, double& y) {
    std::lock_guard<std::mutex> lock(odom_mu_);
    x = odom_x_;
    y = odom_y_;
  }

  double get_yaw() {
    std::lock_guard<std::mutex> lock(odom_mu_);
    return odom_yaw_;
  }

  static double wrap_angle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::mutex odom_mu_;
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_yaw_ = 0.0;
  bool odom_received_ = false;
};

// =====================================================================
// RobosotFiraMission — full mission orchestration
// =====================================================================
class RobosotFiraMission {
public:
  RobosotFiraMission(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<rclcpp::Logger> logger,
    Commander1& commander,
    NavHelper& nav,
    const moveit::core::LinkModel* ee_link)
    : node_(node), logger_(logger), commander_(&commander),
      nav_(&nav), ee_link_(ee_link)
  {}

  void run() {
    static constexpr int MAX_PASSES = 3;

    std::vector<int> pending;  // indices into LETTERS
    for (size_t i = 0; i < LETTERS.size(); ++i) pending.push_back(static_cast<int>(i));
    int placed_count = 0;

    // ---- PHASE 1: initial navigation to HOME ----
    // Sequence: arm:north → backward → arm:left → strafe LEFT → backward.
    RCLCPP_INFO(*logger_, "======== PHASE 1: Navigate to HOME ========");
    if (!commander_->move_gripper("open")) {
      RCLCPP_WARN(*logger_, "Initial gripper open failed. Continuing.");
    }
    if (!commander_->move_arm_to_named(SEARCH_POSE)) {
      RCLCPP_ERROR(*logger_, "Failed to move arm to '%s'. Aborting.", SEARCH_POSE.c_str());
      return;
    }
    nav_->drive_straight(-INIT_PRE_BACKWARD_M);

    if (!commander_->move_arm_to_named(ARM_LEFT_POSE)) {
      RCLCPP_ERROR(*logger_, "Failed to move arm to '%s'. Aborting.", ARM_LEFT_POSE.c_str());
      return;
    }
    // +y = LEFT in REP-103.
    nav_->strafe(+INIT_STRAFE_RIGHT_M);
    nav_->drive_straight(-INIT_BACKWARD_M);
    RCLCPP_INFO(*logger_, "======== HOME reached ========");

    // ---- PHASE 2: visit each goods area, search for any pending letter ----
    // 3 areas × 4 possible positions = 12 block positions total.
    // Visit each area, scan for every pending letter. If found, pick it,
    // bring it home, place it, then return to the same area to look for more.
    for (int pass = 1; pass <= MAX_PASSES && !pending.empty() && rclcpp::ok(); ++pass) {
      RCLCPP_INFO(*logger_, "############### PASS %d/%d (%zu letters pending) ###############",
                  pass, MAX_PASSES, pending.size());

      for (size_t spoke_idx = 0; spoke_idx < GOODS_SPOKES.size() && !pending.empty() && rclcpp::ok();
           ++spoke_idx)
      {
        const auto& spoke = GOODS_SPOKES[spoke_idx];
        RCLCPP_INFO(*logger_, "===== AREA `%s` (spoke %zu) =====", spoke.name.c_str(), spoke_idx);

        // Navigate home → goods area.
        commander_->move_arm_to_named(ARM_LEFT_POSE);
        navigate_to_goods(spoke_idx);

        // Search for each pending letter at this area.
        // Keep scanning until no more letters are found here.
        bool found_any_this_visit = true;
        while (found_any_this_visit && !pending.empty() && rclcpp::ok()) {
          found_any_this_visit = false;

          // Move arm to search pose once per scan sweep.
          if (!commander_->move_arm_to_named(SEARCH_POSE)) break;

          // Try each pending letter — pick the first one YOLO sees.
          int found_idx = -1;
          for (int li : pending) {
            const auto& letter = LETTERS[li];
            const std::string topic = ERROR_TOPIC_PREFIX + to_lower(letter);
            subscribe_error(topic);
            RCLCPP_INFO(*logger_, "[%s@%s] Scanning for `%s`...",
                        spoke.name.c_str(), letter.c_str(), topic.c_str());

            if (wait_fresh_error(DETECTION_TIMEOUT)) {
              RCLCPP_INFO(*logger_, "[%s] DETECTED at area `%s`!", letter.c_str(), spoke.name.c_str());
              found_idx = li;
              break;
            }
          }

          if (found_idx < 0) {
            RCLCPP_INFO(*logger_, "No pending letters found at `%s`. Moving on.", spoke.name.c_str());
            break;
          }

          // Found a letter — align + pick + bring home + place.
          const auto& letter = LETTERS[found_idx];
          const auto& place_pose = PLACE_POSES[found_idx];
          found_any_this_visit = true;

          // ALIGN
          if (!align()) {
            RCLCPP_WARN(*logger_, "[%s] Align failed. Skipping this letter for now.", letter.c_str());
            continue;
          }

          // PICK: open → descend → camera offset → close
          RCLCPP_INFO(*logger_, "[%s] PICK", letter.c_str());
          if (!commander_->move_gripper("open")) continue;
          if (!descend()) continue;
          if (!apply_camera_offset()) continue;
          if (!commander_->move_gripper("close")) continue;
          std::this_thread::sleep_for(POST_ACTION_DELAY);
          if (!go_up()) continue;

          // TRANSIT: search → transit → left (for navigation camera)
          if (!commander_->move_arm_to_named(SEARCH_POSE)) continue;
          if (!commander_->move_arm_to_named(TRANSIT_POSE)) continue;
          if (!commander_->move_arm_to_named(ARM_LEFT_POSE)) continue;

          // NAVIGATE goods → home
          RCLCPP_INFO(*logger_, "[%s] NAV: goods → home", letter.c_str());
          navigate_to_home(spoke_idx);

          // PLACE: transit → place pose → release
          RCLCPP_INFO(*logger_, "[%s] PLACE → `%s`", letter.c_str(), place_pose.c_str());
          commander_->move_arm_to_named(TRANSIT_POSE);
          if (!commander_->move_arm_to_named(place_pose)) {
            RCLCPP_WARN(*logger_, "[%s] Place pose failed. Letter may be lost.", letter.c_str());
            commander_->move_gripper("open");
            continue;
          }
          commander_->move_gripper("open");
          std::this_thread::sleep_for(POST_ACTION_DELAY);
          retract();
          std::this_thread::sleep_for(POST_ACTION_DELAY);
          go_up();
          commander_->move_arm_to_named(TRANSIT_POSE);

          // Mark letter as placed.
          pending.erase(std::remove(pending.begin(), pending.end(), found_idx), pending.end());
          ++placed_count;
          RCLCPP_INFO(*logger_, "Letter `%s` placed! (%zu remaining)", letter.c_str(), pending.size());

          // If more letters pending, navigate back to same goods area to continue scanning.
          if (!pending.empty()) {
            RCLCPP_INFO(*logger_, "Returning to `%s` to search for more letters.", spoke.name.c_str());
            commander_->move_arm_to_named(ARM_LEFT_POSE);
            navigate_to_goods(spoke_idx);
          }
        }

        // Done with this area — return home before moving to next spoke.
        // (Only navigate home if we're still AT the goods area, i.e. the last
        //  iteration didn't find anything and we didn't already go home.)
        if (!found_any_this_visit) {
          commander_->move_arm_to_named(ARM_LEFT_POSE);
          navigate_to_home(spoke_idx);
        }
      }
    }

    RCLCPP_INFO(*logger_, "======== Mission complete. placed=%d skipped=%zu of %zu ========",
                placed_count, pending.size(), LETTERS.size());
    for (int i : pending) {
      RCLCPP_WARN(*logger_, "  Skipped letter: %s", LETTERS[i].c_str());
    }
  }

private:
  // ----- Navigation helpers -----

  bool navigate_to_goods(size_t spoke_idx) {
    const Spoke& spoke = GOODS_SPOKES[spoke_idx];
    RCLCPP_INFO(*logger_, "NAV: home → `%s`", spoke.name.c_str());
    if (spoke.use_strafe) {
      nav_->strafe(spoke.strafe_m);
    } else {
      nav_->turn(spoke.turn_deg * M_PI / 180.0);
    }
    nav_->drive_straight(spoke.drive_m);
    nav_->drive_straight(spoke.approach_m);
    return true;
  }

  bool navigate_to_home(size_t spoke_idx) {
    const Spoke& spoke = GOODS_SPOKES[spoke_idx];
    RCLCPP_INFO(*logger_, "NAV: `%s` → home", spoke.name.c_str());
    nav_->drive_straight(spoke.retreat_m);
    nav_->drive_straight(-spoke.drive_m);
    if (spoke.use_strafe) {
      nav_->strafe(-spoke.strafe_m);
    } else {
      nav_->turn(-spoke.turn_deg * M_PI / 180.0);
    }
    return true;
  }

  // ----- YOLO alignment (same logic as fira_mission_yolo) -----

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
      RCLCPP_INFO(*logger_, "Align iter %d: err=(%+.1f, %+.1f) px", i, ex, ey);

      if (std::abs(ex) < PIXEL_THRESHOLD && std::abs(ey) < PIXEL_THRESHOLD) {
        RCLCPP_INFO(*logger_, "Alignment converged.");
        return true;
      }

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
      auto current_pose = arm.getCurrentPose("ee_link").pose;

      geometry_msgs::msg::Pose target_pose = current_pose;
      target_pose.position.x += dx;
      target_pose.position.y += dy;

      commander_->clear_target_n_constraints();
      arm.setStartStateToCurrentState();

      if (!commander_->move_ptp(target_pose, "AlignStep", ee_link_)) {
        RCLCPP_WARN(*logger_, "Align step failed.");
        return false;
      }

      if (!wait_fresh_error(ERROR_WAIT_TIMEOUT)) {
        RCLCPP_WARN(*logger_, "No fresh error after move.");
        return false;
      }
    }
    RCLCPP_WARN(*logger_, "Align iteration cap reached.");
    return false;
  }

  bool apply_camera_offset() {
    auto& arm = commander_->arm_move_group();
    auto start_pose = arm.getCurrentPose("ee_link").pose;

    const double final_x = start_pose.position.x + CAMERA_OFFSET_X;
    const double final_y = start_pose.position.y + CAMERA_OFFSET_Y;

    geometry_msgs::msg::Pose sub_target = start_pose;
    int sub_iter = 0;
    while (rclcpp::ok()) {
      double rx = final_x - sub_target.position.x;
      double ry = final_y - sub_target.position.y;
      if (std::abs(rx) < 1e-6 && std::abs(ry) < 1e-6) break;

      sub_target.position.x += std::clamp(rx, -STEP_XY, STEP_XY);
      sub_target.position.y += std::clamp(ry, -STEP_XY, STEP_XY);

      constexpr int kMaxAttempts = 3;
      bool sub_ok = false;
      for (int a = 1; a <= kMaxAttempts && rclcpp::ok(); ++a) {
        commander_->clear_target_n_constraints();
        arm.setStartStateToCurrentState();
        if (commander_->move_ptp(sub_target, "CameraOffset", ee_link_)) { sub_ok = true; break; }
        std::this_thread::sleep_for(250ms);
      }
      if (!sub_ok) return false;
      std::this_thread::sleep_for(150ms);
      ++sub_iter;
    }
    return true;
  }

  bool descend() {
    auto& arm = commander_->arm_move_group();
    auto cp = arm.getCurrentPose("ee_link").pose;
    geometry_msgs::msg::Pose tp = cp;
    tp.position.z += STEP_Z;
    commander_->clear_target_n_constraints();
    arm.setStartStateToCurrentState();
    return commander_->move_ptp(tp, "Descend", ee_link_);
  }

  bool go_up() {
    auto& arm = commander_->arm_move_group();
    auto cp = arm.getCurrentPose("ee_link").pose;
    geometry_msgs::msg::Pose tp = cp;
    tp.position.z += GO_UP_Z;
    commander_->clear_target_n_constraints();
    arm.setStartStateToCurrentState();
    return commander_->move_ptp(tp, "GoUp", ee_link_);
  }

  bool retract() {
    auto& arm = commander_->arm_move_group();
    auto cp = arm.getCurrentPose("ee_link").pose;
    geometry_msgs::msg::Pose tp = cp;
    tp.position.y += RETRACT_Y;
    commander_->clear_target_n_constraints();
    arm.setStartStateToCurrentState();
    return commander_->move_ptp(tp, "Retract", ee_link_);
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
  NavHelper* nav_;
  const moveit::core::LinkModel* ee_link_;

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
  auto const node = std::make_shared<rclcpp::Node>(
    "robosot_fira_mission",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("robosot_fira_mission");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  auto move_group_interface = MoveGroupInterface(node, "arm");
  auto gripper_group_interface = MoveGroupInterface(node, "gripper");
  move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group_interface.setPlannerId("PTP");

  auto visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel()};
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // Floor collision object
  {
    moveit::planning_interface::PlanningSceneInterface psi;
    moveit_msgs::msg::CollisionObject floor;
    floor.header.frame_id = move_group_interface.getPlanningFrame();
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

  Commander1 commander(logger_ptr, move_group_interface, gripper_group_interface,
                       visual_tools, params);
  NavHelper nav(node, logger_ptr);

  RCLCPP_INFO(logger, "Waiting for odometry...");
  if (!nav.wait_odom()) {
    RCLCPP_ERROR(logger, "No odometry received. Aborting.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  auto const ee_link = move_group_interface.getRobotModel()->getLinkModel("ee_link");
  RobosotFiraMission mission(node, logger_ptr, commander, nav, ee_link);
  mission.run();

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
