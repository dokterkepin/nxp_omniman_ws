from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_path = FindPackageShare("omniman_navigation")

    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Use simulation time from Isaac Sim",
    )

    sim_time = {"use_sim_time": LaunchConfiguration("use_sim")}

    slam_config = PathJoinSubstitution(
        [pkg_path, "config", "slam_toolbox.yaml"]
    )

    rviz_config = PathJoinSubstitution(
        [pkg_path, "config", "slam_toolbox_config.rviz"]
    )

    # --- Odometry ---
    #
    # Matches nav2_launch.py: wheel odometry only. rf2o (laser odometry) and the
    # robot_localization EKF have both been removed, so mecanum_drive_controller
    # is the sole source of odom -> base_footprint and publishes that transform
    # itself (enable_odom_tf: true in controllers.yaml).
    #
    # This must stay aligned with nav2_launch.py: a map built against laser
    # odometry would not match what wheel odometry produces at run time.
    #
    #   mapping:    odom --(mecanum_drive_controller)--> base_footprint
    #               map  --(slam_toolbox)-------------> odom
    #   navigation: same odom link, map -> odom from AMCL instead
    #
    # No odometry node is launched here - it comes from ros2_control, which is
    # already running as part of the robot bringup.

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_config, sim_time],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[sim_time],
    )

    return LaunchDescription([
        use_sim_arg,
        slam_toolbox_node,
        rviz_node,
    ])
