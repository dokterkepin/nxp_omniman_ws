from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


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

    # --- slam_toolbox ---
    #
    # Since slam_toolbox 2.8 (Jazzy) this is a LIFECYCLE node. Started as a
    # plain Node it sits in "unconfigured" forever: it never subscribes to
    # /scan, never publishes /map or map -> odom, and RViz (fixed frame map)
    # drops every scan with "Message Filter dropping message: frame
    # 'lidar_link' ... queue is full". So configure it on launch and activate
    # it once configuring finishes - the same as slam_toolbox's own
    # online_async_launch.py.

    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[slam_config, sim_time, {"use_lifecycle_manager": False}],
    )

    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="slam_toolbox configured - activating"),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
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
        configure_slam,
        activate_slam,
        rviz_node,
    ])
