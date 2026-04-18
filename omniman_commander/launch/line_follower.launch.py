from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("image_topic", default_value="/image_raw"),
        DeclareLaunchArgument("cmd_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("forward_speed", default_value="0.06"),
        DeclareLaunchArgument("approach_speed", default_value="0.02"),
        DeclareLaunchArgument("max_lateral_speed", default_value="0.08"),
        DeclareLaunchArgument("use_strafe", default_value="true"),
        DeclareLaunchArgument("kp", default_value="0.8"),
        DeclareLaunchArgument("ki", default_value="0.0"),
        DeclareLaunchArgument("kd", default_value="0.05"),
        DeclareLaunchArgument("error_sign", default_value="-1.0"),
        DeclareLaunchArgument("black_thresh", default_value="70"),
        DeclareLaunchArgument("roi_top_frac", default_value="0.55"),
        DeclareLaunchArgument("red_area_stop_frac", default_value="0.22"),
        DeclareLaunchArgument("red_area_slow_frac", default_value="0.05"),
        DeclareLaunchArgument("stop_and_exit", default_value="true"),
        DeclareLaunchArgument("show_window", default_value="true"),
    ]

    node = Node(
        package="omniman_commander",
        executable="line_follower_pid.py",
        name="line_follower_pid",
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "cmd_topic": LaunchConfiguration("cmd_topic"),
                "forward_speed": LaunchConfiguration("forward_speed"),
                "approach_speed": LaunchConfiguration("approach_speed"),
                "max_lateral_speed": LaunchConfiguration("max_lateral_speed"),
                "use_strafe": LaunchConfiguration("use_strafe"),
                "kp": LaunchConfiguration("kp"),
                "ki": LaunchConfiguration("ki"),
                "kd": LaunchConfiguration("kd"),
                "error_sign": LaunchConfiguration("error_sign"),
                "black_thresh": LaunchConfiguration("black_thresh"),
                "roi_top_frac": LaunchConfiguration("roi_top_frac"),
                "red_area_stop_frac": LaunchConfiguration("red_area_stop_frac"),
                "red_area_slow_frac": LaunchConfiguration("red_area_slow_frac"),
                "stop_and_exit": LaunchConfiguration("stop_and_exit"),
                "show_window": LaunchConfiguration("show_window"),
            }
        ],
    )

    return LaunchDescription([*args, node])
