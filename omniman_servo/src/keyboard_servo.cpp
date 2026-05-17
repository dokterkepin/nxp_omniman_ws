#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <termios.h>
#include <unistd.h>
#include <signal.h>

static struct termios old_termios;
static bool running = true;

void restore_terminal(int)
{
  tcsetattr(STDIN_FILENO, TCSANOW, &old_termios);
  running = false;
}

char get_key()
{
  char c = 0;
  struct timeval tv = {0, 100000};  // 100ms timeout
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);
  if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0)
    read(STDIN_FILENO, &c, 1);
  return c;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("keyboard_servo");

  auto twist_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", 10);
  auto joint_pub = node->create_publisher<control_msgs::msg::JointJog>(
      "/servo_node/delta_joint_cmds", 10);

  // Set terminal to raw mode
  tcgetattr(STDIN_FILENO, &old_termios);
  struct termios raw = old_termios;
  raw.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  signal(SIGINT, restore_terminal);

  RCLCPP_INFO(node->get_logger(), "=== OmniMan Servo Keyboard Control ===");
  RCLCPP_INFO(node->get_logger(), "Cartesian mode (TwistStamped):");
  RCLCPP_INFO(node->get_logger(), "  w/s : +X / -X");
  RCLCPP_INFO(node->get_logger(), "  a/d : +Y / -Y");
  RCLCPP_INFO(node->get_logger(), "  q/e : +Z / -Z");
  RCLCPP_INFO(node->get_logger(), "  i/k : +Roll / -Roll");
  RCLCPP_INFO(node->get_logger(), "  j/l : +Pitch / -Pitch");
  RCLCPP_INFO(node->get_logger(), "  u/o : +Yaw / -Yaw");
  RCLCPP_INFO(node->get_logger(), "Joint mode (JointJog):");
  RCLCPP_INFO(node->get_logger(), "  1-6 : jog joint[n] positive");
  RCLCPP_INFO(node->get_logger(), "  Shift+1-6 (!/@ etc.) : jog joint[n] negative");
  RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to quit");

  const std::vector<std::string> joint_names = {
      "shoulder_yaw_joint", "upper_shoulder_pitch_joint", "arm_yaw_joint",
      "forearm_pitch_joint", "wrist_pitch_joint", "palm_yaw_joint"};

  while (running && rclcpp::ok()) {
    char key = get_key();
    if (key == 0)
      continue;

    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = node->now();
    twist_msg.header.frame_id = "base_link";

    auto joint_msg = control_msgs::msg::JointJog();
    joint_msg.header.stamp = node->now();

    bool send_twist = false;
    bool send_joint = false;

    switch (key) {
      // Linear
      case 'w': twist_msg.twist.linear.x = 1.0; send_twist = true; break;
      case 's': twist_msg.twist.linear.x = -1.0; send_twist = true; break;
      case 'a': twist_msg.twist.linear.y = 1.0; send_twist = true; break;
      case 'd': twist_msg.twist.linear.y = -1.0; send_twist = true; break;
      case 'q': twist_msg.twist.linear.z = 1.0; send_twist = true; break;
      case 'e': twist_msg.twist.linear.z = -1.0; send_twist = true; break;
      // Angular
      case 'i': twist_msg.twist.angular.x = 1.0; send_twist = true; break;
      case 'k': twist_msg.twist.angular.x = -1.0; send_twist = true; break;
      case 'j': twist_msg.twist.angular.y = 1.0; send_twist = true; break;
      case 'l': twist_msg.twist.angular.y = -1.0; send_twist = true; break;
      case 'u': twist_msg.twist.angular.z = 1.0; send_twist = true; break;
      case 'o': twist_msg.twist.angular.z = -1.0; send_twist = true; break;
      // Joint positive (1-6)
      case '1': case '2': case '3': case '4': case '5': case '6':
        joint_msg.joint_names.push_back(joint_names[key - '1']);
        joint_msg.velocities.push_back(1.0);
        send_joint = true;
        break;
      // Joint negative (shift+1-6)
      case '!':
        joint_msg.joint_names.push_back(joint_names[0]);
        joint_msg.velocities.push_back(-1.0); send_joint = true; break;
      case '@':
        joint_msg.joint_names.push_back(joint_names[1]);
        joint_msg.velocities.push_back(-1.0); send_joint = true; break;
      case '#':
        joint_msg.joint_names.push_back(joint_names[2]);
        joint_msg.velocities.push_back(-1.0); send_joint = true; break;
      case '$':
        joint_msg.joint_names.push_back(joint_names[3]);
        joint_msg.velocities.push_back(-1.0); send_joint = true; break;
      case '%':
        joint_msg.joint_names.push_back(joint_names[4]);
        joint_msg.velocities.push_back(-1.0); send_joint = true; break;
      case '^':
        joint_msg.joint_names.push_back(joint_names[5]);
        joint_msg.velocities.push_back(-1.0); send_joint = true; break;
    }

    if (send_twist)
      twist_pub->publish(twist_msg);
    if (send_joint)
      joint_pub->publish(joint_msg);
  }

  tcsetattr(STDIN_FILENO, TCSANOW, &old_termios);
  rclcpp::shutdown();
  return 0;
}
