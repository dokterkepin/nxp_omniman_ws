"""run_mission(): everything around the tree - ROS, the mission file, the
startup checks, the tick loop, the log, and stopping the robot on any exit."""

import time

import py_trees
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator
from omniman_interfaces.srv import ReleaseControl
from py_trees.common import Status
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger

from .robot import NAV, Robot, load_poses, make_pose
from .step import Step
from .steps import Align

# Internal: messages handled per tick at most. Odometry alone arrives faster
# than the tick rate; reading one message per tick lets them pile up and the
# steps judge stale data. Nothing to tune.
SPIN_PER_TICK = 50


# mission.yaml `settings:` keys this package needs.
SETTINGS = ['tick_s', 'log_every_s', 'service_wait_s', 'still_time_s', 'still_linear',
            'still_angular', 'settle_timeout_s']


def check_align_targets(root):
    """Every Align in the tree must name exactly a prompt of the detector
    visual_align listens to: detections carry the prompt as their name, and
    visual_align accepts only the current target, so a target spelled
    differently ("black square" vs "black rectangle") is never found and the
    run fails after a full search turn. Returns what is wrong, or ''."""
    targets = sorted({b.target for b in root.iterate() if isinstance(b, Align)})
    if not targets:
        return ''
    path = f"{get_package_share_directory('omniman_vla')}/config/visual_align.yaml"
    with open(path) as f:
        va = yaml.safe_load(f) or {}
    topic = va.get('visual_align', {}).get('detections_topic', '')
    detector = topic.strip('/').split('/')[0]        # /efficient_sam_detector/detections
    prompts = list(va.get(detector, {}).get('prompts') or [])
    if not prompts:
        return (f'cannot check the align targets: no prompts for "{detector}" (from '
                f'detections_topic {topic}) in {path}')
    bad = [t for t in targets if t not in prompts]
    if bad:
        return (f'align target(s) {bad} not among {detector}\'s prompts {prompts} - '
                'fix the mission or the prompts in visual_align.yaml')
    return ''


def call_sync(nav, client, request, timeout_s=5.0):
    """A blocking service call - for shutdown only, never inside a step."""
    if not client.wait_for_service(timeout_sec=timeout_s):
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(nav, future, timeout_sec=timeout_s)
    return future.result()


def run_mission(build, node_name, initial_pose='home'):
    """Run the tree build(robot) returns until it succeeds or fails.

    build         function robot -> root behaviour of the mission's tree
    node_name     ROS node name of the mission
    initial_pose  place in poses.yaml to give AMCL at start (the robot is
                  assumed to stand there when launched); None = keep AMCL's

    The mission file is the `mission_file` parameter (default:
    config/mission.yaml); poses.yaml is read from beside it. Returns True
    if the mission succeeded.
    """
    # rclpy's SIGINT handler would shut the context down before the steps
    # could stop what they started; Python's default raises KeyboardInterrupt.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    nav = BasicNavigator(node_name=node_name)
    log = nav.get_logger()

    default_cfg = f"{get_package_share_directory('omniman_vla')}/config/mission.yaml"
    nav.declare_parameter('mission_file', default_cfg)
    mission_file = nav.get_parameter('mission_file').value
    with open(mission_file) as f:
        cfg = yaml.safe_load(f)
    cfg['poses'] = load_poses(mission_file)
    log.info(f'mission: {mission_file}')
    missing = [k for k in SETTINGS if k not in (cfg.get('settings') or {})]
    if missing:
        log.error(f'{mission_file} settings: missing {missing}')
        nav.destroy_node()
        rclpy.try_shutdown()
        return False

    robot = Robot(nav, cfg)
    root = build(robot)
    problem = check_align_targets(root)
    if problem:
        log.error(problem)
        nav.destroy_node()
        rclpy.try_shutdown()
        return False

    if initial_pose is not None:
        # BasicNavigator's default initial pose is a zero-norm quaternion, and
        # waitUntilNav2Active() publishes it until AMCL answers - so a real one
        # must be set, or a good AMCL estimate is clobbered.
        nav.setInitialPose(make_pose(nav, cfg['poses'][initial_pose]))
    log.info('waiting for Nav2...')
    nav.waitUntilNav2Active()

    tree = py_trees.trees.BehaviourTree(root)
    tick_s = float(cfg['settings']['tick_s'])
    log_every_s = float(cfg['settings']['log_every_s'])
    shown, last_line = None, 0.0
    try:
        while True:
            start = time.monotonic()
            tree.tick()
            # No tree in the log: each step logs its own result, and while it
            # runs, one line every log_every_s about what it is waiting for.
            # A step changing status restarts that interval.
            statuses = tuple(n.status for n in root.iterate())
            if statuses != shown:
                shown, last_line = statuses, time.monotonic()
            else:
                running = root.tip()
                if (isinstance(running, Step)
                        and time.monotonic() - last_line >= log_every_s):
                    log.info(f'   {running.name}: {running.progress()}')
                    last_line = time.monotonic()
            if root.status in (Status.SUCCESS, Status.FAILURE):
                break
            # Handle every message waiting, not just one; each call returns
            # at once when nothing is waiting.
            for _ in range(SPIN_PER_TICK):
                rclpy.spin_once(nav, timeout_sec=0.0)
            time.sleep(max(0.0, tick_s - (time.monotonic() - start)))
        if root.status == Status.SUCCESS:
            log.info('mission complete')
        else:
            log.error('mission ABORTED')
    except KeyboardInterrupt:
        log.warn('interrupted - stopping everything')
        root.stop(Status.INVALID)
    finally:
        # Never leave the robot driving, aligning, running a policy or
        # holding "nav"; each stop is a no-op when that part is idle.
        nav.cancelTask()
        call_sync(nav, robot.align_stop, Trigger.Request())
        call_sync(nav, robot.policy_stop, Trigger.Request())
        if robot.owner == NAV:
            req = ReleaseControl.Request()
            req.owner = NAV
            call_sync(nav, robot.release_client, req)
        nav.destroy_node()
        rclpy.try_shutdown()
    return root.status == Status.SUCCESS
