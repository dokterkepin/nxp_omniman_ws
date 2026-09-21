#!/usr/bin/env python3
"""
Finds coloured targets in the wrist camera image - no neural network.

    in   /image_raw/compressed            sensor_msgs/CompressedImage
    out  ~/detections                     vision_msgs/Detection2DArray
         ~/debug/compressed               sensor_msgs/CompressedImage   targets drawn

Each entry of `targets` is a colour range in HSV (OpenCV: H 0-180, S and V
0-255) plus a size and shape check. Per frame, every target's largest blob
that passes its checks becomes one detection, class_id = the target's name,
score 1.0. Detections are published in the order of `targets`, so the first
target in the list that is in view comes first - and visual_align aligns to
the first detection. Order the list by priority.

Why colour and not YOLO: on the recorded frames the yellow lid was found in
173 of 173 aligned frames (YOLO-World: 147), centred within 1 px of YOLO's box,
at 0.4 ms a frame on the CPU. The lid is overexposed on this camera - what
sets it apart from the cardboard is hue (27-30 vs 19-22) and brightness (>210
vs ~172), not saturation. The black mark and container are the only near-black
(V ~34) areas on bright cardboard (V ~170); the shape check keeps the black
aluminium rails out.

Inference only runs while something subscribes to ~/detections or the debug
image. Plain OpenCV, no GPU: runs on the robot PC (control_launch.py).

Settings: config/visual_align.yaml, section color_detector - read from the
file directly (no ROS parameters). Saved edits (targets, colour ranges)
apply within a second; image_topic only at start.

Tune with the debug image:
  ros2 run rqt_image_view rqt_image_view /color_detector/debug/compressed
"""

import os
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# Settings every target needs, under its name in the config file.
TARGET_SETTINGS = ['hsv_low', 'hsv_high', 'min_area', 'min_fill', 'max_aspect']


def missing_settings(values):
    """Names missing from the color_detector section: image_topic, targets,
    and each listed target's five settings."""
    missing = [k for k in ('image_topic', 'targets') if k not in values]
    for t in values.get('targets') or []:
        block = values.get(t)
        if not isinstance(block, dict):
            missing.append(t)
        else:
            missing += [f'{t}.{k}' for k in TARGET_SETTINGS if k not in block]
    return missing


# Debug colours (BGR), cycled per target.
DEBUG_COLORS = [(0, 0, 255), (0, 200, 0), (255, 0, 0), (0, 165, 255), (255, 0, 255)]

CONFIG_FILE = os.path.join(get_package_share_directory('omniman_vla'), 'config',
                           'visual_align.yaml')


class Config:
    """This node's section of config/visual_align.yaml - read straight from
    the file, not through ROS parameters. Re-read when the file changes
    (checked at most once a second), so saved edits apply without a restart;
    an edit that breaks the file is logged and the previous values are kept."""

    def __init__(self, section, missing, logger):
        # missing(values) -> names of the settings the section lacks.
        self.section, self.missing, self.logger = section, missing, logger
        self.checked = 0.0
        self.load()

    def load(self):
        mtime = os.stat(CONFIG_FILE).st_mtime
        with open(CONFIG_FILE) as f:
            values = (yaml.safe_load(f) or {}).get(self.section) or {}
        missing = self.missing(values)
        if missing:
            raise RuntimeError(f'{CONFIG_FILE} [{self.section}] is missing: '
                               f'{", ".join(missing)}')
        self.values, self.mtime = values, mtime

    def __getitem__(self, key):
        now = time.monotonic()
        if now - self.checked > 1.0:
            self.checked = now
            try:
                if os.stat(CONFIG_FILE).st_mtime != self.mtime:
                    self.load()
                    self.logger.info(f'reloaded {CONFIG_FILE}')
            except (OSError, yaml.YAMLError, RuntimeError, AttributeError) as e:
                self.logger.error(f'bad edit, keeping previous values: {e}')
                self.mtime = os.stat(CONFIG_FILE).st_mtime
        return self.values[key]


class ColorDetector(Node):

    def __init__(self):
        super().__init__('color_detector')
        # Every value comes from config/visual_align.yaml [color_detector].
        self.cfg = Config('color_detector', missing_settings, self.get_logger())

        self.det_pub = self.create_publisher(Detection2DArray, '~/detections', 10)
        self.debug_pub = self.create_publisher(CompressedImage, '~/debug/compressed', 1)
        self.create_subscription(
            CompressedImage, self.cfg['image_topic'],
            self.on_image, qos_profile_sensor_data)
        self.kernel = np.ones((5, 5), np.uint8)
        self.get_logger().info(f'ready - targets in priority order: {self.cfg["targets"]}')

    def p(self, target, setting):
        return self.cfg[target][setting]

    def mask(self, hsv, target):
        """Pixels inside the target's HSV range. hsv_low H > hsv_high H wraps
        around 180 (for red)."""
        lo = [int(v) for v in self.p(target, 'hsv_low')]
        hi = [int(v) for v in self.p(target, 'hsv_high')]
        if lo[0] <= hi[0]:
            m = cv2.inRange(hsv, tuple(lo), tuple(hi))
        else:
            m = (cv2.inRange(hsv, (lo[0], lo[1], lo[2]), (180, hi[1], hi[2]))
                 | cv2.inRange(hsv, (0, lo[1], lo[2]), (hi[0], hi[1], hi[2])))
        return cv2.morphologyEx(m, cv2.MORPH_OPEN, self.kernel)

    def find(self, hsv, target):
        """Largest blob of this target that passes its checks, or None.

        min_area    px, smaller blobs are noise
        min_fill    blob area / its rotated bounding box area (0 = off); a
                    solid square is ~0.9, a thin bent rail much less
        max_aspect  long side / short side of that box (0 = off)
        """
        contours, _ = cv2.findContours(self.mask(hsv, target), cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        min_fill = float(self.p(target, 'min_fill'))
        max_aspect = float(self.p(target, 'max_aspect'))
        best, best_area = None, float(self.p(target, 'min_area'))
        for c in contours:
            area = cv2.contourArea(c)
            if area < best_area:
                continue
            (_, _), (w, h), _ = cv2.minAreaRect(c)
            if min(w, h) < 1:
                continue
            if min_fill > 0 and area / (w * h) < min_fill:
                continue
            if max_aspect > 0 and max(w, h) / min(w, h) > max_aspect:
                continue
            best, best_area = c, area
        return best

    def on_image(self, msg):
        want_det = self.det_pub.get_subscription_count() > 0
        want_debug = self.debug_pub.get_subscription_count() > 0
        if not (want_det or want_debug):
            return
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        out = Detection2DArray()
        out.header = msg.header
        found = []
        targets = list(self.cfg['targets'])
        for target in targets:
            c = self.find(hsv, target)
            if c is None:
                continue
            x, y, w, h = cv2.boundingRect(c)
            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = x + w / 2.0
            det.bbox.center.position.y = y + h / 2.0
            det.bbox.size_x = float(w)
            det.bbox.size_y = float(h)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = target
            hyp.hypothesis.score = 1.0
            det.results.append(hyp)
            out.detections.append(det)
            found.append((target, c))
        self.det_pub.publish(out)

        if want_debug:
            for i, (target, c) in enumerate(found):
                color = DEBUG_COLORS[targets.index(target) % len(DEBUG_COLORS)]
                x, y, w, h = cv2.boundingRect(c)
                cv2.drawContours(frame, [c], -1, color, 2)
                cv2.circle(frame, (x + w // 2, y + h // 2), 5, color, -1)
                label = f'{i + 1}. {target}' + ('  <- align' if i == 0 else '')
                cv2.putText(frame, label, (x, max(y - 6, 16)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            dbg = CompressedImage()
            dbg.header = msg.header
            dbg.format = 'jpeg'
            dbg.data = cv2.imencode('.jpg', frame)[1].tobytes()
            self.debug_pub.publish(dbg)


def main():
    rclpy.init()
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
