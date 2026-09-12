"""
Touch UI for Nav2 (web/index.html), for driving the robot from an iPad.

    iPad browser --http:8081--> this static page
                 --ws:9091----> rosbridge --> ROS 2

This stack is independent of physical_ai_tools, which runs its own rosbridge
on 9090 and web_video_server on 8080.

Both servers run on the machine that runs Nav2, so open
http://<this-pc-ip>:8081 on the iPad. Plain http on purpose: Safari refuses a
ws:// connection from an https page, which is what blocks app.foxglove.dev.

Run alongside nav2_launch.py:
  ros2 launch omniman_navigation web_ui_launch.py
  ros2 launch omniman_navigation web_ui_launch.py web_port:=8888
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    web_dir = PathJoinSubstitution([FindPackageShare('omniman_navigation'), 'web'])

    # 8081, not 8080: physical_ai_tools' web_video_server holds 8080, and
    # python's http.server would just fail to bind.
    web_port = DeclareLaunchArgument(
        'web_port', default_value='8081',
        description='Port the page is served on')

    # 9091, not the usual 9090: physical_ai_server_bringup.launch.py runs its
    # own rosbridge there, and these two stacks stay independent - either can
    # run without the other.
    rosbridge_port = DeclareLaunchArgument(
        'rosbridge_port', default_value='9091',
        description='Websocket port the page talks ROS over')

    # The page connects to ws://<its own host>:9091, so this must listen on
    # every interface, not just localhost.
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'address': '0.0.0.0',
            'port': LaunchConfiguration('rosbridge_port'),
        }],
    )

    # Static files only - no build step, no framework.
    web_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', LaunchConfiguration('web_port'),
             '--bind', '0.0.0.0', '--directory', web_dir],
        output='screen',
    )

    return LaunchDescription([
        web_port,
        rosbridge_port,
        rosbridge,
        web_server,
    ])
