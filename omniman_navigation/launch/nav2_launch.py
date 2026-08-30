from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_path = FindPackageShare("omniman_navigation")
    nav2_bringup = FindPackageShare("nav2_bringup")

    map_file = DeclareLaunchArgument(
        "map",
        default_value=PathJoinSubstitution(
            [pkg_path, "maps", "my_map_v3.yaml"]
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

    params_file = LaunchConfiguration("params_file")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key="",
            param_rewrites={
                "use_sim_time": "false",
                "autostart": "true",
                "yaml_filename": LaunchConfiguration("map"),
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # --- Localization (map_server + amcl) ---

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [nav2_bringup, "launch", "localization_launch.py"]
            )
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "params_file": params_file,
            "use_sim_time": "false",
            "autostart": "true",
        }.items(),
    )

    # controller_server publishes /cmd_vel directly - nothing sits between it
    # and the robot.
    #
    #   controller_server --> /cmd_vel --> twist_to_twist_stamped
    #                                  --> /cmd_vel_stamped --> mecanum controller
    #
    # NOTE: with no velocity_smoother, NOTHING in the stack limits acceleration.
    # Humble's MPPI has no acceleration constraints (ControlConstraints is
    # {vx_max, vx_min, vy, wz} only) and mecanum_drive_controller implements no
    # limits either (unlike diff_drive_controller, which has
    # has_acceleration_limits / max_acceleration / max_jerk). So a commanded
    # step from 0 to vx_max reaches the wheels in one 50 ms control tick.
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        parameters=[configured_params],
        remappings=remappings,
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[configured_params],
        remappings=remappings,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[configured_params],
        remappings=remappings,
    )

    # Provides /navigate_to_pose, which BasicNavigator (nav2_simple_commander)
    # connects to. Uses Nav2's stock behaviour tree - no custom XML.
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[configured_params],
        remappings=remappings,
    )

    # node_names must list exactly the lifecycle nodes that are actually running.
    # Naming one that was never launched leaves the manager waiting on it forever
    # and the stack never reaches active.
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": [
                "bt_navigator",
                "controller_server",
                "planner_server",
                "behavior_server",
            ],
        }],
    )

    # --- Twist → TwistStamped relay for mecanum controller ---

    twist_relay = Node(
        package="omniman_navigation",
        executable="twist_to_twist_stamped.py",
        name="twist_to_twist_stamped",
        output="screen",
    )

    # --- RViz ---

    rviz_config = PathJoinSubstitution(
        [pkg_path, "config", "nav2_config.rviz"]
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
        localization_launch,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
        twist_relay,
        rviz_node,
    ])
