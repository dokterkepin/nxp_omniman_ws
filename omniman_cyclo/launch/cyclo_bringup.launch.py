#!/usr/bin/env python3
"""
Cyclo Intelligence (src/cyclo_intelligence) with omniman's layer, natively -
the counterpart of physical_ai_server_bringup.launch.py:

    ros2 launch omniman_cyclo cyclo_bringup.launch.py
    ros2 launch omniman_cyclo cyclo_bringup.launch.py robot_type:=omniman_mobile

Starts Cyclo's supervisor (its UI backend) and web UI on http://<this pc>:7080,
and with them the orchestrator (+ rosbridge, rosbag recorder, web_video_server),
cyclo_data and the behaviour-tree engine for `robot_type`. The UI's buttons
stop and start those as in Cyclo's container. Ctrl+C stops everything.

Once, before the first launch: colcon build, then
src/omniman_cyclo/native/install.sh (Python packages, the web UI, data folder).

ROS_DOMAIN_ID / RMW_IMPLEMENTATION are the shell's. Runtime files - Python
environment, logs, recordings - are in nxp_omniman_ws/cyclo/ (outside git).
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration


def paths():
    """Workspace paths. With --symlink-install the installed scripts link back
    into src/; otherwise the workspace is two levels above the install prefix."""
    native = Path(get_package_share_directory('omniman_cyclo')) / 'native'
    src = (native / 'web.py').resolve().parents[2]
    if not (src / 'cyclo_intelligence').is_dir():
        src = Path(get_package_prefix('omniman_cyclo')).parents[1] / 'src'
    ws = src.parent
    home = Path(os.environ.get('CYCLO_HOME', ws / 'cyclo'))
    return native, src, ws, home


def setup(context):
    native, src, ws, home = paths()
    cyclo = src / 'cyclo_intelligence'
    venv_site = home / 'venv' / 'lib' / 'python3.12' / 'site-packages'
    if not venv_site.is_dir() or not (cyclo / 'orchestrator' / 'ui' / 'build').is_dir():
        return [LogInfo(msg=f'[cyclo] not installed yet - run {src}/omniman_cyclo/native/'
                            'install.sh, then launch again')]
    workspace = home / 'workspace'
    env = {
        # Cyclo's Python packages for its processes only - its ROS nodes run
        # on the system Python - as Cyclo's image does with /opt/venv.
        'PYTHONPATH': os.pathsep.join(
            p for p in (str(venv_site), str(cyclo / 'docker'), os.environ.get('PYTHONPATH'))
            if p),
        'PATH': f'{home / "venv" / "bin"}{os.pathsep}{os.environ.get("PATH", "")}',
        'PYTHONUNBUFFERED': '1',
        'OMNIMAN_SRC': str(src),
        'CYCLO_DIR': str(cyclo),
        'CYCLO_HOME': str(home),
        'COLCON_WS': str(ws),                       # Cyclo defaults to /root/ros2_ws
        'CYCLO_WORKSPACE': str(workspace),          # = /workspace, see install.sh
        'CYCLO_BT_TREES_DIR': str(workspace / 'bt' / 'trees'),
        'CYCLO_BT_EXAMPLE_TREES_DIR': str(ws / 'install' / 'orchestrator' / 'share'
                                          / 'orchestrator' / 'bt' / 'trees'),
        'CYCLO_NAVIGATION_DATA_DIR': str(workspace / 'navigation'),
        'CYCLO_ROBOT_CONFIGS_DIR': str(cyclo / 'shared' / 'shared' / 'robot_configs'),
        'ORCHESTRATOR_CONFIG_PATH': str(cyclo / 'shared' / 'shared' / 'robot_configs'),
        'CYCLO_ROBOT_TYPE': LaunchConfiguration('robot_type').perform(context),
        'CYCLO_AUTOSTART': LaunchConfiguration('autostart').perform(context),
        'CYCLO_UI_PORT': LaunchConfiguration('ui_port').perform(context),
    }
    actions = [SetEnvironmentVariable(k, v) for k, v in env.items()]
    if os.path.realpath('/workspace') != os.path.realpath(workspace):
        actions.append(LogInfo(msg=f'[cyclo] /workspace is not {workspace} - recording and '
                                   f'models need it: sudo ln -sfn {workspace} /workspace'))
    actions += [
        ExecuteProcess(cmd=['python3', str(native / 'supervisor_native.py')],
                       name='cyclo_supervisor', output='screen', sigterm_timeout='20'),
        ExecuteProcess(cmd=['python3', str(native / 'web.py')],
                       name='cyclo_web', output='screen'),
        LogInfo(msg=f'[cyclo] UI: http://localhost:{env["CYCLO_UI_PORT"]} '
                    f'(ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")}, '
                    f'{os.environ.get("RMW_IMPLEMENTATION", "default RMW")})'),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='omniman',
                              description='robot type for the behaviour-tree engine '
                                          '(omniman, omniman_mobile)'),
        DeclareLaunchArgument('autostart', default_value='orchestrator,cyclo_data,bt_node',
                              description='services to start right away; "" = from the UI'),
        DeclareLaunchArgument('ui_port', default_value='7080'),
        OpaqueFunction(function=setup),
    ])
