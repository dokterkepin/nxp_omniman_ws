// Autonomous distance driver for the mecanum base.
//
// Publish a geometry_msgs/Point to /drive_goal where x and y are the desired
// displacement in METERS, expressed in the robot's body frame at the moment
// the goal is received. The node closes the loop on
// /mecanum_drive_controller/odometry and drives a TwistStamped to /cmd_vel
// until the goal is reached, then stops.

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;

class DriveDistance : public rclcpp::Node
{
public:
  DriveDistance() : rclcpp::Node("drive_distance")
  {
    max_linear_vel_ = declare_parameter<double>("max_linear_vel", 0.25);
    kp_ = declare_parameter<double>("kp", 1.2);
    position_tolerance_ = declare_parameter<double>("position_tolerance", 0.02);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 50.0);

    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mecanum_drive_controller/odometry", rclcpp::SensorDataQoS(),
      std::bind(&DriveDistance::onOdom, this, _1));

    goal_sub_ = create_subscription<geometry_msgs::msg::Point>(
      "/drive_goal", 1, std::bind(&DriveDistance::onGoal, this, _1));

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&DriveDistance::controlStep, this));

    RCLCPP_INFO(get_logger(),
      "drive_distance ready. Publish geometry_msgs/Point to /drive_goal "
      "(x,y in meters, robot body frame).");
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

  void onGoal(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (!current_) {
      RCLCPP_WARN(get_logger(), "Goal received but no odometry yet — ignoring.");
      return;
    }
    start_ = *current_;
    goal_body_ = Pose2D{msg->x, msg->y, 0.0};
    active_ = true;
    RCLCPP_INFO(get_logger(),
      "New goal: dx=%.3f m, dy=%.3f m (body frame at start yaw=%.3f rad).",
      msg->x, msg->y, start_->yaw);
  }

  void publishStop()
  {
    geometry_msgs::msg::TwistStamped t;
    t.header.stamp = now();
    t.header.frame_id = "base_link";
    cmd_pub_->publish(t);
  }

  void controlStep()
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (!active_ || !current_ || !start_) return;

    // Displacement since start, in odom frame.
    const double dx_odom = current_->x - start_->x;
    const double dy_odom = current_->y - start_->y;

    // Rotate into the body frame at the moment the goal was latched, so that
    // the (x, y) commanded by the user stays aligned with the robot's forward
    // / left axes even if odom is not axis-aligned with the body.
    const double c = std::cos(-start_->yaw);
    const double s = std::sin(-start_->yaw);
    const double dx_body = c * dx_odom - s * dy_odom;
    const double dy_body = s * dx_odom + c * dy_odom;

    const double err_x = goal_body_.x - dx_body;
    const double err_y = goal_body_.y - dy_body;
    const double err_norm = std::hypot(err_x, err_y);

    if (err_norm < position_tolerance_) {
      publishStop();
      active_ = false;
      RCLCPP_INFO(get_logger(), "Goal reached (residual %.3f m). Stopping.", err_norm);
      return;
    }

    // P-controller with a shared cap so direction is preserved when saturating.
    double vx = kp_ * err_x;
    double vy = kp_ * err_y;
    const double v_norm = std::hypot(vx, vy);
    if (v_norm > max_linear_vel_) {
      const double scale = max_linear_vel_ / v_norm;
      vx *= scale;
      vy *= scale;
    }

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";
    cmd.twist.linear.x = vx;
    cmd.twist.linear.y = vy;
    cmd_pub_->publish(cmd);
  }

  // Parameters.
  double max_linear_vel_;
  double kp_;
  double position_tolerance_;
  double control_rate_hz_;

  // IO.
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // State.
  std::mutex mu_;
  std::optional<Pose2D> current_;
  std::optional<Pose2D> start_;
  Pose2D goal_body_{0.0, 0.0, 0.0};
  bool active_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveDistance>());
  rclcpp::shutdown();
  return 0;
}
