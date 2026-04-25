#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <algorithm>
#include <iostream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/point.hpp>

using namespace std::chrono_literals;
namespace rvt = rviz_visual_tools;
using moveit::planning_interface::MoveGroupInterface;

// ---------- Mission tuning (adjust here) ----------
static constexpr double STEP_XY           = 0.01;   // MAX step per alignment iteration (meters)
static constexpr double P_GAIN            = 1e-4;   // meters per pixel of error (P control: small err -> small step)
static constexpr double STEP_Z            = -0.14;  // meters to descend after align
static constexpr double GO_UP_Z           = 0.1;   // meters to lift ee after pick/place (relative +Z)
static constexpr double RETRACT_Y         = 0.02;   // meters to pull back along +Y (backward from east side) after release, before lifting
static constexpr auto   POST_ACTION_DELAY = 500ms;   // pause after close/release before moving
static constexpr double PIXEL_THRESHOLD   = 15.0;   // alignment done when |err| < this
static constexpr int    MAX_ALIGN_ITERS   = 30;     // safety cap
static constexpr auto   ERROR_WAIT_TIMEOUT = 3s;    // wait at most this for a fresh error (mid-align)
static constexpr auto   DETECTION_TIMEOUT  = 5s;    // wait at most this for FIRST detection per letter; skip if exceeded
static const std::string ERROR_TOPIC_PREFIX = "/yolo_error_";   // per-letter topic = prefix + lowercase(letter)

// ---- Sequential FIRA mission configuration ----
static const std::vector<std::string> LETTERS = {"F", "I", "R", "A"};
static const std::string SEARCH_POSE = "north";  // searched-for-target pose (shared across letters)
static const std::string EAST_POSE   = "east";   // waypoint between pick and place
// Place poses: SRDF named targets with the same name as each letter ("F", "I", "R", "A")

// Physical offset from ee_link to the camera optical axis.
// Applied as an XY move after alignment (camera-centered) to put the gripper over the target.
// Flip signs if the gripper ends up on the wrong side of the target.
static constexpr double CAMERA_OFFSET_X   = 0.05;   // meters
static constexpr double CAMERA_OFFSET_Y   = 0.0;    // meters

// Maps pixel error -> robot XY step direction.
// Flip a sign if the robot moves the wrong way during the first test.
static constexpr double SIGN_ERR_X_TO_DY  = -1.0;   // err_x > 0 (target right in image) -> dy = -step
static constexpr double SIGN_ERR_Y_TO_DX  = -1.0;   // err_y > 0 (target below in image) -> dx = -step
// ---------------------------------------------------

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
    std::cout << std::string(125, '*') << std::endl;
    RCLCPP_INFO(*logger_, "Move to `%s`: pos=(%.4f, %.4f, %.4f)", text.c_str(),
      target_pose.position.x, target_pose.position.y, target_pose.position.z);

    clear_target_n_constraints();
    arm_move_group_->setPlannerId("PTP");
    arm_move_group_->setMaxVelocityScalingFactor(params_->arm_max_velocity_scaling_factor);
    arm_move_group_->setMaxAccelerationScalingFactor(params_->arm_max_acceleration_scaling_factor);

    auto current_state = arm_move_group_->getCurrentState();
    arm_move_group_->setStartState(*current_state);
    arm_move_group_->setPoseTarget(target_pose, params_->end_effector_link);

    std::string text_lower = text;
    std::transform(text_lower.begin(), text_lower.end(), text_lower.begin(), ::tolower);

    auto plan_result = arm_move_group_->plan(plan_);
    is_plan_success_ = plan_result == moveit::core::MoveItErrorCode::SUCCESS;
    RCLCPP_INFO(*logger_, "Plan: %s", is_plan_success_ ? "succeeded" : "failed");

    visual_tools_->publishAxisLabeled(target_pose, text_lower);
    visual_tools_->publishText(text_pose_, text, rvt::WHITE, rvt::XLARGE);

    is_execution_success_ = false;
    if (is_plan_success_) {
      visual_tools_->publishTrajectoryLine(plan_.trajectory_, ee_link, joint_model_group_);
      visual_tools_->trigger();

      auto execution_result = arm_move_group_->execute(plan_);
      is_execution_success_ = execution_result == moveit::core::MoveItErrorCode::SUCCESS;
      RCLCPP_INFO(*logger_, "Execute: %s", is_execution_success_ ? "succeeded" : "failed");

      if (!is_execution_success_) {
        RCLCPP_ERROR(*logger_, "Execute failed: %d (%s)", static_cast<int>(execution_result.val),
                     moveit::core::error_code_to_string(execution_result).c_str());
      }
    } else {
      RCLCPP_WARN(*logger_, "Plan failed: %d (%s)", static_cast<int>(plan_result.val),
                  moveit::core::error_code_to_string(plan_result).c_str());
    }

    return is_plan_success_ && is_execution_success_;
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
        RCLCPP_WARN(*logger_, "Gripper plan failed for `%s` (attempt %d/%d)",
                    named_target.c_str(), attempt, kMaxAttempts);
        std::this_thread::sleep_for(kRetryDelay);
        continue;
      }
      auto exec_result = gripper_move_group_->execute(gplan);
      if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
        return true;
      }
      RCLCPP_WARN(*logger_, "Gripper execute failed for `%s` (attempt %d/%d): %d",
                  named_target.c_str(), attempt, kMaxAttempts,
                  static_cast<int>(exec_result.val));
      std::this_thread::sleep_for(kRetryDelay);
    }
    RCLCPP_ERROR(*logger_, "Gripper `%s` gave up after %d attempts.",
                 named_target.c_str(), kMaxAttempts);
    return false;
  }

  MoveGroupInterface& arm_move_group() { return *arm_move_group_; }

private:
  std::shared_ptr<rclcpp::Logger> logger_;
  MoveGroupInterface* arm_move_group_;
  MoveGroupInterface* gripper_move_group_;
  moveit_visual_tools::MoveItVisualTools* visual_tools_;
  CommanderParams* params_;

  MoveGroupInterface::Plan plan_;
  bool is_plan_success_ = false;
  bool is_execution_success_ = false;
  Eigen::Isometry3d text_pose_;
  const moveit::core::JointModelGroup* joint_model_group_;
};

class FiraMission {
public:
  FiraMission(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<rclcpp::Logger> logger,
    Commander1& commander,
    const moveit::core::LinkModel* ee_link)
    : node_(node), logger_(logger), commander_(&commander), ee_link_(ee_link)
  {
    // Subscription is created per-letter inside process_letter().
  }

  void run() {
    // Multi-pass orchestration:
    //   - Placed     → letter done, remove from pending.
    //   - Failed     → retry immediately (up to MAX_ATTEMPTS attempts per pass), IK may work on retry.
    //   - NoDetection→ skip to next letter NOW, but keep in pending for the next pass.
    //                   This lets the system come back to a letter that was briefly invisible due to
    //                   YOLO noise, after giving the frame time to recover.
    //
    // Mission never aborts: after MAX_PASSES passes, any still-pending letters are reported skipped.
    static constexpr int MAX_ATTEMPTS = 2;
    static constexpr int MAX_PASSES   = 3;

    std::vector<std::string> pending(LETTERS.begin(), LETTERS.end());
    int placed_count = 0;

    RCLCPP_INFO(*logger_, "Mission start: opening gripper.");
    if (!commander_->move_gripper("open")) {
      RCLCPP_WARN(*logger_, "Initial gripper open failed. Continuing anyway.");
    }

    for (int pass = 1; pass <= MAX_PASSES && !pending.empty() && rclcpp::ok(); ++pass) {
      RCLCPP_INFO(*logger_, "######################### PASS %d/%d (%zu pending) #########################",
                  pass, MAX_PASSES, pending.size());

      std::vector<std::string> still_pending;
      for (const auto& letter : pending) {
        RCLCPP_INFO(*logger_, "========================= LETTER `%s` =========================", letter.c_str());

        bool placed = false;
        bool detection_miss = false;

        for (int attempt = 1; attempt <= MAX_ATTEMPTS && rclcpp::ok(); ++attempt) {
          RCLCPP_INFO(*logger_, "[%s] Pass %d Attempt %d/%d", letter.c_str(), pass, attempt, MAX_ATTEMPTS);

          // On retries within a pass, reset to east first.
          if (attempt > 1) {
            if (!commander_->move_arm_to_named(EAST_POSE)) {
              RCLCPP_WARN(*logger_, "[%s] Could not reach home pose `%s` before retry. Proceeding anyway.",
                          letter.c_str(), EAST_POSE.c_str());
            }
          }

          auto result = process_letter(letter);
          if (result == LetterResult::Placed) {
            placed = true;
            break;
          }
          if (result == LetterResult::NoDetection) {
            // Don't retry immediately — defer for a later pass when YOLO may see the letter again.
            detection_miss = true;
            break;
          }
          // result == Failed — retry in next attempt of this pass.
          RCLCPP_WARN(*logger_, "[%s] Attempt %d/%d failed (IK/plan). Will retry.",
                      letter.c_str(), attempt, MAX_ATTEMPTS);
        }

        if (placed) {
          RCLCPP_INFO(*logger_, "Letter `%s` complete.", letter.c_str());
          ++placed_count;
        } else {
          const char* reason = detection_miss ? "no-detection" : "IK/plan failure";
          RCLCPP_WARN(*logger_, "Letter `%s` deferred (%s) — will re-try next pass.", letter.c_str(), reason);
          still_pending.push_back(letter);
        }
      }

      pending = still_pending;
    }

    RCLCPP_INFO(*logger_, "FiraMission complete. placed=%d skipped=%zu of %zu letters.",
                placed_count, pending.size(), LETTERS.size());
    for (const auto& s : pending) {
      RCLCPP_WARN(*logger_, "  Skipped letter: %s", s.c_str());
    }
  }

private:
  enum class LetterResult {
    Placed,        // success
    NoDetection,   // YOLO didn't publish within DETECTION_TIMEOUT — defer, don't retry immediately
    Failed         // IK / planning / execution failure — worth retrying right away
  };

  LetterResult process_letter(const std::string& letter) {
    // STATE 1 — SEARCH: move arm to shared search pose.
    RCLCPP_INFO(*logger_, "[%s] STATE: SEARCH -> moving to `%s`", letter.c_str(), SEARCH_POSE.c_str());
    if (!commander_->move_arm_to_named(SEARCH_POSE)) return LetterResult::Failed;

    // STATE 2 — ALIGN: subscribe to this letter's error topic and converge.
    const std::string topic = ERROR_TOPIC_PREFIX + to_lower(letter);
    subscribe_error(topic);
    RCLCPP_INFO(*logger_, "[%s] STATE: ALIGN -> waiting for first `%s` message...", letter.c_str(), topic.c_str());
    if (!wait_fresh_error(DETECTION_TIMEOUT)) {
      RCLCPP_WARN(*logger_, "[%s] Not detected on `%s` within %lds. Deferring letter for retry pass.",
                  letter.c_str(), topic.c_str(),
                  std::chrono::duration_cast<std::chrono::seconds>(DETECTION_TIMEOUT).count());
      return LetterResult::NoDetection;
    }
    if (!align()) return LetterResult::Failed;

    // STATE 3 — PICK: open → descend (-Z) → camera offset (+X) → close.
    // Descend MUST happen before the offset: at high Z the arm can't reach the offset
    // target (not enough horizontal reach). Descending first re-shapes the elbow so the
    // lateral move is reachable. Offset sub-steps include retries for transient
    // CONTROL_FAILED at low Z.
    RCLCPP_INFO(*logger_, "[%s] STATE: PICK (open -> descend -> offset -> close)", letter.c_str());
    if (!commander_->move_gripper("open")) return LetterResult::Failed;
    if (!descend()) return LetterResult::Failed;
    if (!apply_camera_offset()) return LetterResult::Failed;
    if (!commander_->move_gripper("close")) return LetterResult::Failed;

    // STATE 4 — POST-PICK: delay, then lift +Z to clear the object.
    RCLCPP_INFO(*logger_, "[%s] STATE: POST-PICK (delay -> go up)", letter.c_str());
    std::this_thread::sleep_for(POST_ACTION_DELAY);
    if (!go_up()) return LetterResult::Failed;

    // STATE 5 — TRANSIT: via north, then east, on the way to the place pose.
    RCLCPP_INFO(*logger_, "[%s] STATE: TRANSIT -> `%s` -> `%s`", letter.c_str(),
                SEARCH_POSE.c_str(), EAST_POSE.c_str());
    if (!commander_->move_arm_to_named(SEARCH_POSE)) return LetterResult::Failed;
    if (!commander_->move_arm_to_named(EAST_POSE)) return LetterResult::Failed;

    // STATE 6 — PLACE: move to letter's place pose, then open to release.
    RCLCPP_INFO(*logger_, "[%s] STATE: PLACE -> moving to named target `%s`", letter.c_str(), letter.c_str());
    if (!commander_->move_arm_to_named(letter)) return LetterResult::Failed;
    if (!commander_->move_gripper("open")) return LetterResult::Failed;

    // STATE 7 — POST-PLACE: delay → retract → delay → lift → back to east.
    RCLCPP_INFO(*logger_, "[%s] STATE: POST-PLACE (delay -> retract -> go up -> back to `%s`)",
                letter.c_str(), EAST_POSE.c_str());
    std::this_thread::sleep_for(POST_ACTION_DELAY);
    if (!retract()) return LetterResult::Failed;
    std::this_thread::sleep_for(POST_ACTION_DELAY);
    if (!go_up()) return LetterResult::Failed;
    if (!commander_->move_arm_to_named(EAST_POSE)) return LetterResult::Failed;

    return LetterResult::Placed;
  }

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

  static std::string to_lower(const std::string& s) {
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
  }

private:
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
        RCLCPP_INFO(*logger_, "Alignment within threshold. Done aligning.");
        return true;
      }

      // Proportional step: magnitude = min(P_GAIN * |err|, STEP_XY).
      // Big error -> clamped at STEP_XY. Small error -> tiny step, so it can't overshoot past zero.
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

      RCLCPP_INFO(*logger_,
        "Current pose:  pos=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)",
        current_pose.position.x, current_pose.position.y, current_pose.position.z,
        current_pose.orientation.x, current_pose.orientation.y,
        current_pose.orientation.z, current_pose.orientation.w);
      RCLCPP_INFO(*logger_, "Step delta:    (dx, dy) = (%+.4f, %+.4f)", dx, dy);

      geometry_msgs::msg::Pose target_pose;
      target_pose.position.x = current_pose.position.x + dx;
      target_pose.position.y = current_pose.position.y + dy;
      target_pose.position.z = current_pose.position.z;
      target_pose.orientation.x = current_pose.orientation.x;
      target_pose.orientation.y = current_pose.orientation.y;
      target_pose.orientation.z = current_pose.orientation.z;
      target_pose.orientation.w = current_pose.orientation.w;

      RCLCPP_INFO(*logger_,
        "Target pose:   pos=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)",
        target_pose.position.x, target_pose.position.y, target_pose.position.z,
        target_pose.orientation.x, target_pose.orientation.y,
        target_pose.orientation.z, target_pose.orientation.w);

      commander_->clear_target_n_constraints();
      arm.setStartStateToCurrentState();

      if (!commander_->move_ptp(target_pose, "AlignStep", ee_link_)) {
        auto joints = arm.getCurrentJointValues();
        auto names = arm.getJointNames();
        RCLCPP_ERROR(*logger_, "Joint values at failure:");
        for (size_t k = 0; k < joints.size() && k < names.size(); ++k) {
          RCLCPP_ERROR(*logger_, "  %s = %.4f rad", names[k].c_str(), joints[k]);
        }
        RCLCPP_WARN(*logger_, "Align step failed. Aborting.");
        return false;
      }

      if (!wait_fresh_error(ERROR_WAIT_TIMEOUT)) {
        RCLCPP_WARN(*logger_, "No fresh error after move. Aborting.");
        return false;
      }
    }
    RCLCPP_WARN(*logger_, "Align iteration cap reached without convergence.");
    return false;
  }

  bool apply_camera_offset() {
    // Split the offset into sub-steps of at most STEP_XY to stay inside the workspace.
    // All sub-targets are precomputed from the ORIGINAL start pose, not from getCurrentPose
    // each iteration — avoids cumulative overshoot: each small controller overshoot would
    // otherwise push the arm further forward than commanded, stacking up across sub-steps
    // until IK fails at the workspace edge.
    auto& arm = commander_->arm_move_group();
    auto start_pose = arm.getCurrentPose("ee_link").pose;

    const double final_x = start_pose.position.x + CAMERA_OFFSET_X;
    const double final_y = start_pose.position.y + CAMERA_OFFSET_Y;

    geometry_msgs::msg::Pose sub_target = start_pose;

    int sub_iter = 0;
    while (rclcpp::ok()) {
      double remaining_x = final_x - sub_target.position.x;
      double remaining_y = final_y - sub_target.position.y;
      if (std::abs(remaining_x) < 1e-6 && std::abs(remaining_y) < 1e-6) break;

      double step_x = std::clamp(remaining_x, -STEP_XY, STEP_XY);
      double step_y = std::clamp(remaining_y, -STEP_XY, STEP_XY);

      sub_target.position.x += step_x;
      sub_target.position.y += step_y;
      // z and orientation inherited from start_pose, kept constant

      RCLCPP_INFO(*logger_, "CameraOffset sub-iter %d: step=(%+.4f, %+.4f), target=(%.4f, %.4f), remaining=(%+.4f, %+.4f)",
                  sub_iter, step_x, step_y, sub_target.position.x, sub_target.position.y,
                  remaining_x - step_x, remaining_y - step_y);

      constexpr int kMaxAttempts = 3;
      constexpr auto kRetryDelay = 250ms;
      bool sub_ok = false;
      for (int attempt = 1; attempt <= kMaxAttempts && rclcpp::ok(); ++attempt) {
        commander_->clear_target_n_constraints();
        arm.setStartStateToCurrentState();
        if (commander_->move_ptp(sub_target, "CameraOffset", ee_link_)) {
          sub_ok = true;
          break;
        }
        RCLCPP_WARN(*logger_, "CameraOffset sub-iter %d attempt %d/%d failed; retrying.",
                    sub_iter, attempt, kMaxAttempts);
        std::this_thread::sleep_for(kRetryDelay);
      }
      if (!sub_ok) {
        RCLCPP_WARN(*logger_, "CameraOffset sub-step gave up after %d successful sub-iters.", sub_iter);
        return false;
      }

      // Brief settle so joint state catches up before the next sub-step starts.
      // Without this the next trajectory's start state can mismatch the controller's
      // current state -> CONTROL_FAILED in ~50 ms.
      std::this_thread::sleep_for(150ms);
      ++sub_iter;
    }

    return true;
  }

  bool descend() {
    auto& arm = commander_->arm_move_group();
    auto current_pose = arm.getCurrentPose("ee_link").pose;

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = current_pose.position.x;
    target_pose.position.y = current_pose.position.y;
    target_pose.position.z = current_pose.position.z + STEP_Z;
    target_pose.orientation.x = current_pose.orientation.x;
    target_pose.orientation.y = current_pose.orientation.y;
    target_pose.orientation.z = current_pose.orientation.z;
    target_pose.orientation.w = current_pose.orientation.w;

    commander_->clear_target_n_constraints();
    arm.setStartStateToCurrentState();

    return commander_->move_ptp(target_pose, "Descend", ee_link_);
  }

  bool go_up() {
    auto& arm = commander_->arm_move_group();
    auto current_pose = arm.getCurrentPose("ee_link").pose;

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = current_pose.position.x;
    target_pose.position.y = current_pose.position.y;
    target_pose.position.z = current_pose.position.z + GO_UP_Z;
    target_pose.orientation.x = current_pose.orientation.x;
    target_pose.orientation.y = current_pose.orientation.y;
    target_pose.orientation.z = current_pose.orientation.z;
    target_pose.orientation.w = current_pose.orientation.w;

    commander_->clear_target_n_constraints();
    arm.setStartStateToCurrentState();

    return commander_->move_ptp(target_pose, "GoUp", ee_link_);
  }

  bool retract() {
    // Retract "backward" — since we approach place poses from the east side of the robot,
    // backward is +Y (not -X).
    auto& arm = commander_->arm_move_group();
    auto current_pose = arm.getCurrentPose("ee_link").pose;

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = current_pose.position.x;
    target_pose.position.y = current_pose.position.y + RETRACT_Y;
    target_pose.position.z = current_pose.position.z;
    target_pose.orientation.x = current_pose.orientation.x;
    target_pose.orientation.y = current_pose.orientation.y;
    target_pose.orientation.z = current_pose.orientation.z;
    target_pose.orientation.w = current_pose.orientation.w;

    commander_->clear_target_n_constraints();
    arm.setStartStateToCurrentState();

    return commander_->move_ptp(target_pose, "Retract", ee_link_);
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::Logger> logger_;
  Commander1* commander_;
  const moveit::core::LinkModel* ee_link_;

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr error_sub_;

  std::mutex error_mu_;
  std::condition_variable error_cv_;
  double latest_err_x_ = 0.0;
  double latest_err_y_ = 0.0;
  bool error_fresh_ = false;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "fira_mission_yolo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("fira_mission_yolo");

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
  }

  CommanderParams params;
  auto logger_ptr = std::make_shared<rclcpp::Logger>(logger);
  Commander1 commander(logger_ptr, move_group_interface, gripper_group_interface, visual_tools, params);

  auto const ee_link = move_group_interface.getRobotModel()->getLinkModel("ee_link");

  FiraMission mission(node, logger_ptr, commander, ee_link);
  mission.run();

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
