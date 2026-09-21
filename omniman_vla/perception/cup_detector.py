#!/usr/bin/env python3
"""
Finds the cup in the wrist camera image.

    in   /image_raw/compressed            sensor_msgs/CompressedImage
    out  ~/detections                     vision_msgs/Detection2DArray  best first
         ~/debug/compressed               sensor_msgs/CompressedImage   boxes drawn

YOLO-World, prompted with text instead of trained on our cup. Stock COCO YOLO
only finds this cup (white, yellow lid, seen from above) in 12-35% of recorded
frames - it calls it a frisbee, cake or toilet. YOLO-World prompted with
"cup with yellow lid" finds it in 98% of the aligned frames of
omniman_base_correct_v6, 0.85-0.99 confidence, ~4 ms a frame on the GPU.

Inference only runs while something subscribes to ~/detections or the debug
image, so the node can stay up next to a running policy without using the GPU.
Old frames are dropped rather than queued: only the newest image is detected.

Needs torch + ultralytics + clip, so run it in the lerobot_jazzy env, on the
GPU PC:
  conda activate lerobot_jazzy && source install/setup.bash
  ros2 run omniman_vla cup_detector.py

Settings: config/visual_align.yaml, section cup_detector (model and classes
only at start).

Check what it sees:
  ros2 run rqt_image_view rqt_image_view /cup_detector/debug/compressed
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
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

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


class CupDetector(Node):

    def __init__(self):
        super().__init__('cup_detector')
        # Every value comes from config/visual_align.yaml [cup_detector].
        self.cfg = Config('cup_detector',
                          lambda v: [k for k in ('model', 'classes', 'image_topic',
                                                 'min_score', 'device') if k not in v],
                          self.get_logger())
        path = os.path.expanduser(self.cfg['model'])
        classes = list(self.cfg['classes'])
        self.device = self.cfg['device']

        self.model = YOLO(path)
        if classes:
            self.model.set_classes(classes)
        # The first call is slow (CUDA init); do it now, not on the first frame.
        self.model(np.zeros((480, 640, 3), np.uint8), device=self.device, verbose=False)

        self.det_pub = self.create_publisher(Detection2DArray, '~/detections', 10)
        self.debug_pub = self.create_publisher(CompressedImage, '~/debug/compressed', 1)
        # Depth 1, best effort: a slow frame drops the ones behind it.
        self.create_subscription(
            CompressedImage, self.cfg['image_topic'],
            self.on_image, qos_profile_sensor_data)

        self.get_logger().info(f'ready - {os.path.basename(path)} looking for {classes}')

    def on_image(self, msg):
        want_det = self.det_pub.get_subscription_count() > 0
        want_debug = self.debug_pub.get_subscription_count() > 0
        if not (want_det or want_debug):
            return

        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        result = self.model(frame, conf=float(self.cfg['min_score']), device=self.device,
                            verbose=False)[0]

        boxes = sorted(
            zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist(),
                result.boxes.cls.tolist()),
            key=lambda b: -b[1])

        out = Detection2DArray()
        out.header = msg.header
        for (x1, y1, x2, y2), score, cls in boxes:
            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = (x1 + x2) / 2.0
            det.bbox.center.position.y = (y1 + y2) / 2.0
            det.bbox.size_x = x2 - x1
            det.bbox.size_y = y2 - y1
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = result.names[int(cls)]
            hyp.hypothesis.score = float(score)
            det.results.append(hyp)
            out.detections.append(det)
        self.det_pub.publish(out)

        if want_debug:
            for i, ((x1, y1, x2, y2), score, _) in enumerate(boxes):
                color = (0, 0, 255) if i == 0 else (0, 165, 255)
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                cv2.putText(frame, f'{score:.2f}', (int(x1), max(int(y1) - 5, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            dbg = CompressedImage()
            dbg.header = msg.header
            dbg.format = 'jpeg'
            dbg.data = cv2.imencode('.jpg', frame)[1].tobytes()
            self.debug_pub.publish(dbg)


def main():
    rclpy.init()
    node = CupDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
