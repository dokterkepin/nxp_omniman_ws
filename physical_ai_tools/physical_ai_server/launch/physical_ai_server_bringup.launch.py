#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Dongyun Kim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('physical_ai_server')

    # Robot type the server is configured for as soon as it is up. Without it,
    # nothing works until someone picks the robot in the web UI: START_INFERENCE
    # fails with "'NoneType' object has no attribute 'get_publisher_msg_types'".
    # robot_type:='' leaves it unset, as upstream does.
    robot_type = DeclareLaunchArgument(
        'robot_type', default_value='omniman',
        description="Robot type to configure on launch ('' to skip)")

    # Include physical_ai_server.launch.py
    physical_ai_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'physical_ai_server.launch.py')
        )
    )

    # Rosbridge websocket node
    rosbridge_websocket_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
    )

    # Include rosbag_recorder service_bag_recorder node
    rosbag_recorder_node = Node(
        package='rosbag_recorder',
        executable='service_bag_recorder',
        name='service_bag_recorder',
        output='screen'
    )

    # web_video_server node
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen'
    )

    # `ros2 service call` waits for /set_robot_type to appear, calls it once and
    # exits - no fixed delay needed. Picking a robot in the web UI later still
    # works; it simply configures the server again.
    set_robot_type = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/set_robot_type',
             'physical_ai_interfaces/srv/SetRobotType',
             ['{robot_type: ', LaunchConfiguration('robot_type'), '}']],
        output='screen',
        condition=UnlessCondition(
            PythonExpression(["'", LaunchConfiguration('robot_type'), "' == ''"])),
    )

    return LaunchDescription([
        robot_type,
        physical_ai_server_launch,
        set_robot_type,
        rosbridge_websocket_node,
        rosbag_recorder_node,
        web_video_server_node,
    ])
