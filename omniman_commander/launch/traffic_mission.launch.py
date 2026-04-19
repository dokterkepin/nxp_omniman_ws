from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("image_topic", default_value="/image_raw"),
        DeclareLaunchArgument("cmd_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("odom_topic",
                              default_value="/mecanum_drive_controller/odometry"),
        DeclareLaunchArgument("arm_action",
                              default_value="/arm_controller/follow_joint_trajectory"),
        DeclareLaunchArgument("forward_speed", default_value="0.06"),
        DeclareLaunchArgument("approach_speed", default_value="0.03"),
        DeclareLaunchArgument("coast_speed", default_value="0.03"),
        DeclareLaunchArgument("max_lateral_speed", default_value="0.08"),
        DeclareLaunchArgument("max_angular_speed", default_value="0.6"),
        DeclareLaunchArgument("line_kp", default_value="0.4"),
        DeclareLaunchArgument("line_kd", default_value="0.05"),
        DeclareLaunchArgument("error_sign", default_value="-1.0"),
        DeclareLaunchArgument("black_thresh", default_value="70"),
        DeclareLaunchArgument("single_line_offset", default_value="0.5"),
        DeclareLaunchArgument("lane_gap_frac_min", default_value="0.35"),
        DeclareLaunchArgument("lane_keep_sign", default_value="1.0"),
        DeclareLaunchArgument("roi_top_frac", default_value="0.55"),
        DeclareLaunchArgument("to_intersection_m", default_value="0.40"),
        DeclareLaunchArgument("turn_sign", default_value="1.0"),
        DeclareLaunchArgument("step_timeout_s", default_value="90.0"),
        DeclareLaunchArgument("step_max_distance_m", default_value="3.5"),
        DeclareLaunchArgument("arm_move_time_s", default_value="3.0"),
        DeclareLaunchArgument("detect_frames", default_value="12"),
        DeclareLaunchArgument("detect_min_blob_px", default_value="400"),
        DeclareLaunchArgument("traffic_approach_m", default_value="0.2"),
        DeclareLaunchArgument("post_detect_m", default_value="0.3"),
        DeclareLaunchArgument("show_window", default_value="true"),
    ]

    node = Node(
        package="omniman_commander",
        executable="traffic_mission.py",
        name="traffic_mission",
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "cmd_topic": LaunchConfiguration("cmd_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "arm_action": LaunchConfiguration("arm_action"),
                "forward_speed": LaunchConfiguration("forward_speed"),
                "approach_speed": LaunchConfiguration("approach_speed"),
                "coast_speed": LaunchConfiguration("coast_speed"),
                "max_lateral_speed": LaunchConfiguration("max_lateral_speed"),
                "max_angular_speed": LaunchConfiguration("max_angular_speed"),
                "line_kp": LaunchConfiguration("line_kp"),
                "line_kd": LaunchConfiguration("line_kd"),
                "error_sign": LaunchConfiguration("error_sign"),
                "black_thresh": LaunchConfiguration("black_thresh"),
                "single_line_offset": LaunchConfiguration("single_line_offset"),
                "lane_gap_frac_min": LaunchConfiguration("lane_gap_frac_min"),
                "lane_keep_sign": LaunchConfiguration("lane_keep_sign"),
                "roi_top_frac": LaunchConfiguration("roi_top_frac"),
                "to_intersection_m": LaunchConfiguration("to_intersection_m"),
                "turn_sign": LaunchConfiguration("turn_sign"),
                "step_timeout_s": LaunchConfiguration("step_timeout_s"),
                "step_max_distance_m": LaunchConfiguration("step_max_distance_m"),
                "arm_move_time_s": LaunchConfiguration("arm_move_time_s"),
                "detect_frames": LaunchConfiguration("detect_frames"),
                "detect_min_blob_px": LaunchConfiguration("detect_min_blob_px"),
                "traffic_approach_m": LaunchConfiguration("traffic_approach_m"),
                "post_detect_m": LaunchConfiguration("post_detect_m"),
                "show_window": LaunchConfiguration("show_window"),
            }
        ],
    )

    return LaunchDescription([*args, node])
