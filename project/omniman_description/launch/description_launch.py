from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2"
        )
    ]

    pkg_path = FindPackageShare("omniman_description")
    gui = LaunchConfiguration("gui")

    urdf_file = PathJoinSubstitution([pkg_path, "urdf", "omniman.urdf"])

    robot_description_content = Command(["cat ", urdf_file])    
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution([pkg_path, "config", "config.rviz"])

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="log",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(gui),
        arguments=["-d", rviz_config_file]
    )

    return LaunchDescription(declared_arguments + [rsp_node, gui_node, rviz_node])       