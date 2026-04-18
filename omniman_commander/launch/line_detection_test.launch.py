from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("image_topic", default_value="/image_raw"),
        DeclareLaunchArgument("black_thresh", default_value="70"),
        DeclareLaunchArgument("roi_top_frac", default_value="0.55"),
        DeclareLaunchArgument("red_area_stop_frac", default_value="0.22"),
        DeclareLaunchArgument("red_area_slow_frac", default_value="0.05"),
        DeclareLaunchArgument("show_window", default_value="true"),
    ]

    node = Node(
        package="omniman_commander",
        executable="line_detection_test.py",
        name="line_detection_test",
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "black_thresh": LaunchConfiguration("black_thresh"),
                "roi_top_frac": LaunchConfiguration("roi_top_frac"),
                "red_area_stop_frac": LaunchConfiguration("red_area_stop_frac"),
                "red_area_slow_frac": LaunchConfiguration("red_area_slow_frac"),
                "show_window": LaunchConfiguration("show_window"),
            }
        ],
    )

    return LaunchDescription([*args, node])
