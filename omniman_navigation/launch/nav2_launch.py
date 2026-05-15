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

    # --- Odometry ---

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

    # --- Navigation nodes (no velocity_smoother, no cmd_vel_nav remap) ---

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

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[configured_params],
        remappings=remappings,
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[configured_params],
        remappings=remappings,
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": [
                "controller_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
                "waypoint_follower",
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
        rf2o_node,
        ekf_node,
        localization_launch,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
        twist_relay,
        rviz_node,
    ])
