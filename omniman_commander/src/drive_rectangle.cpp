// Rectangle-trajectory driver for the mecanum base.
//
// Loops a rectangle in the body frame using pure translation (no yaw):
//   +x (side_x) -> +y (side_y) -> -x (side_x) -> -y (side_y), repeat.
// Edges are strafed, so the robot's heading stays constant throughout.

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class MecanumDriver : public rclcpp::Node
{
public:
  MecanumDriver() : rclcpp::Node("drive_rectangle")
  {
    max_linear_vel_ = declare_parameter<double>("max_linear_vel", 0.25);
    kp_ = declare_parameter<double>("kp", 1.2);
    position_tolerance_ = declare_parameter<double>("position_tolerance", 0.02);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 50.0);

    side_x_ = declare_parameter<double>("side_x", 0.5);
    side_y_ = declare_parameter<double>("side_y", 0.5);
    laps_ = declare_parameter<int>("laps", 100);
    dwell_s_ = declare_parameter<double>("dwell_s", 0.5);

    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mecanum_drive_controller/odometry", rclcpp::SensorDataQoS(),
      std::bind(&MecanumDriver::onOdom, this, _1));
  }

  bool waitForOdom(std::chrono::seconds timeout = 5s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    rclcpp::Rate r(50.0);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lk(mu_);
        if (current_) return true;
      }
      r.sleep();
    }
    return false;
  }

  bool driveBody(double dx, double dy)
  {
    Pose2D start;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (!current_) {
        RCLCPP_ERROR(get_logger(), "driveBody called before odometry available.");
        return false;
      }
      start = *current_;
    }

    RCLCPP_INFO(get_logger(),
      "driveBody: dx=%.3f m, dy=%.3f m", dx, dy);

    const double c = std::cos(-start.yaw);
    const double s = std::sin(-start.yaw);
    rclcpp::Rate rate(control_rate_hz_);

    while (rclcpp::ok()) {
      Pose2D now;
      {
        std::lock_guard<std::mutex> lk(mu_);
        if (!current_) continue;
        now = *current_;
      }

      const double dx_odom = now.x - start.x;
      const double dy_odom = now.y - start.y;
      const double dx_body = c * dx_odom - s * dy_odom;
      const double dy_body = s * dx_odom + c * dy_odom;

      const double err_x = dx - dx_body;
      const double err_y = dy - dy_body;
      const double err_norm = std::hypot(err_x, err_y);

      if (err_norm < position_tolerance_) {
        publishStop();
        RCLCPP_INFO(get_logger(), "edge done (residual %.3f m)", err_norm);
        return true;
      }

      double vx = kp_ * err_x;
      double vy = kp_ * err_y;
      const double v_norm = std::hypot(vx, vy);
      if (v_norm > max_linear_vel_) {
        const double scale = max_linear_vel_ / v_norm;
        vx *= scale;
        vy *= scale;
      }

      geometry_msgs::msg::TwistStamped cmd;
      cmd.header.stamp = rclcpp::Node::now();
      cmd.header.frame_id = "base_link";
      cmd.twist.linear.x = vx;
      cmd.twist.linear.y = vy;
      cmd_pub_->publish(cmd);
      rate.sleep();
    }
    publishStop();
    return false;
  }

  void publishStop()
  {
    geometry_msgs::msg::TwistStamped t;
    t.header.stamp = rclcpp::Node::now();
    t.header.frame_id = "base_link";
    cmd_pub_->publish(t);
  }

  double side_x() const { return side_x_; }
  double side_y() const { return side_y_; }
  int laps() const { return laps_; }
  std::chrono::milliseconds dwell() const {
    return std::chrono::milliseconds(static_cast<int>(dwell_s_ * 1000.0));
  }

private:
  struct Pose2D { double x; double y; double yaw; };

  static double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
  {
    tf2::Quaternion tq(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
    return yaw;
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mu_);
    current_ = Pose2D{
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      yawFromQuat(msg->pose.pose.orientation)};
  }

  double max_linear_vel_;
  double kp_;
  double position_tolerance_;
  double control_rate_hz_;
  double side_x_;
  double side_y_;
  int laps_;
  double dwell_s_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::mutex mu_;
  std::optional<Pose2D> current_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MecanumDriver>();
  auto logger = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  if (!node->waitForOdom()) {
    RCLCPP_ERROR(logger, "No odometry received — is the mecanum controller up?");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  const double sx = node->side_x();
  const double sy = node->side_y();
  const int laps = node->laps();
  const auto dwell = node->dwell();

  RCLCPP_INFO(logger,
    "Rectangle: side_x=%.2f m, side_y=%.2f m, laps=%d, dwell=%ld ms",
    sx, sy, laps, static_cast<long>(dwell.count()));

  for (int i = 0; i < laps && rclcpp::ok(); ++i) {
    RCLCPP_INFO(logger, "=== lap %d/%d ===", i + 1, laps);

    if (!node->driveBody( sx,  0.0)) break;   // forward
    std::this_thread::sleep_for(dwell);
    if (!node->driveBody(0.0,  sy)) break;    // strafe left
    std::this_thread::sleep_for(dwell);
    if (!node->driveBody(-sx,  0.0)) break;   // backward
    std::this_thread::sleep_for(dwell);
    if (!node->driveBody(0.0, -sy)) break;    // strafe right
    std::this_thread::sleep_for(dwell);
  }

  RCLCPP_INFO(logger, "Rectangle loop complete.");
  node->publishStop();
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
