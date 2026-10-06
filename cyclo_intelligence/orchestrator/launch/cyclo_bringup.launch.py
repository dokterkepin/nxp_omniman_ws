#!/usr/bin/env python3
"""
Cyclo Intelligence (src/cyclo_intelligence) with omniman's layer, natively -
the counterpart of physical_ai_server_bringup.launch.py:

    ros2 launch orchestrator cyclo_bringup.launch.py
    ros2 launch orchestrator cyclo_bringup.launch.py robot_type:=omniman_mobile

Starts Cyclo's supervisor (its UI backend) and web UI on http://<this pc>:7080,
and with them the orchestrator (+ rosbridge, rosbag recorder, web_video_server),
cyclo_data and the behaviour-tree engine for `robot_type`, and selects
`robot_type` in the orchestrator (what the UI's Home page does). The UI's
buttons stop and start those as in Cyclo's container. Ctrl+C stops everything.

Policies run in the LeRobot backend (LeRobot 0.6, cyclo_lerobot conda env),
started from the UI's Inference / Training pages or by policy_runner.

Once, before the first launch: colcon build, then
src/cyclo_intelligence/native/install.sh (Python packages, the web UI, data folder).

ROS_DOMAIN_ID / RMW_IMPLEMENTATION are the shell's. Runtime files - Python
environment, logs, recordings - are in nxp_omniman_ws/cyclo/ (outside git).
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration


def paths():
    """Workspace paths. With --symlink-install this installed launch file links
    back into src/cyclo_intelligence/orchestrator/launch/; otherwise the
    workspace is two levels above the install prefix."""
    here = Path(__file__).resolve()
    if here.parents[2].name == 'cyclo_intelligence':
        cyclo = here.parents[2]
    else:
        cyclo = Path(get_package_prefix('orchestrator')).parents[1] / 'src' / 'cyclo_intelligence'
    src = cyclo.parent
    ws = src.parent
    home = Path(os.environ.get('CYCLO_HOME', ws / 'cyclo'))
    return cyclo / 'native', src, ws, home


def setup(context):
    native, src, ws, home = paths()
    cyclo = src / 'cyclo_intelligence'
    venv_site = home / 'venv' / 'lib' / 'python3.12' / 'site-packages'
    if not venv_site.is_dir() or not (cyclo / 'orchestrator' / 'ui' / 'build').is_dir():
        return [LogInfo(msg=f'[cyclo] not installed yet - run {cyclo}/native/'
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
        # The arm's 7 joints alone, in the trained order, for the policy and the
        # recorder (robot_configs: state.arm.topic) - /joint_states has the wheels too.
        ExecuteProcess(cmd=['python3', str(native / 'arm_state_relay.py')],
                       name='cyclo_arm_state', output='screen'),
    ]
    if 'orchestrator' in env['CYCLO_AUTOSTART']:
        # Waits for the orchestrator's service, then sets the robot type once.
        actions.append(ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/set_robot_type', 'interfaces/srv/SetRobotType',
                 f"{{robot_type: '{env['CYCLO_ROBOT_TYPE']}'}}"],
            name='cyclo_set_robot_type', output='log'))
    actions += [
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
