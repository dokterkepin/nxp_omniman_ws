from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_path = FindPackageShare("omniman_ros2_control")
    nav2_bringup = FindPackageShare("nav2_bringup")

    ekf_config = PathJoinSubstitution(
        [pkg_path, "config", "ekf.yaml"]
    )

    map_file = DeclareLaunchArgument(
        "map",
        default_value=PathJoinSubstitution(
            [pkg_path, "maps", "my_map.yaml"]
        ),
        description="Full path to the map yaml file",
    )

    nav2_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [pkg_path, "config", "nav2_params.yaml"]
        ),
        description="Full path to the Nav2 params file",
    )

    rf2o_node = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[{
            "laser_scan_topic": "/scan",
            "odom_topic": "/odom_rf2o",
            "publish_tf": False,
            "base_frame_id": "base_footprint",
            "odom_frame_id": "odom",
            "init_pose_from_topic": "",
            "freq": 20.0,
        }],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config],
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [nav2_bringup, "launch", "bringup_launch.py"]
            )
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
            "use_sim_time": "false",
        }.items(),
    )

    rviz_config = PathJoinSubstitution(
        [nav2_bringup, "rviz", "nav2_default_view.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        map_file,
        nav2_params_file,
        rf2o_node,
        ekf_node,
        nav2_bringup_launch,
        rviz_node,
    ])
