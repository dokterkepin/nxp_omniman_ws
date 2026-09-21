#!/usr/bin/env python3
"""
Finds align targets with SAM 3 (Segment Anything 3), prompted with text.

    in   /image_raw/compressed            sensor_msgs/CompressedImage
    out  ~/detections                     vision_msgs/Detection2DArray
         ~/debug/compressed               sensor_msgs/CompressedImage   masks drawn

One detection per prompt that is found (its highest-scoring mask), class_id =
the prompt, in the order of `prompts` - the first prompt in the list that is in
view comes first, and visual_align aligns to the first detection. The
position is the mask's centroid, not its box centre, so a partly hidden or
oddly shaped object still gives its middle.

SAM 3 finds everything matching a short noun phrase and outlines it exactly -
shadows and floor gaps that are merely dark are not matched. On the
recorded frames (RTX 4060 Ti, fp16, imgsz 644): "yellow cup lid" found the cup
in 20/20 aligned frames; "black square" outlined the mark cleanly (0.64-0.97)
and ignored the shadow at the box edge. Avoid "black mark": it
matched dirt specks all over the box. ~100 ms a frame for one prompt, ~115 ms
for two, 3.4 GB of GPU memory; imgsz 1008 (SAM 3's native size) is ~3x slower
for the same scores here.

Needs the GPU and the lerobot_jazzy env (torch + ultralytics), and the SAM 3
weights: sam3.pt from https://huggingface.co/facebook/sam3 (gated - request
access, then `hf download facebook/sam3 sam3.pt --local-dir ~/models`).

Inference only runs while something subscribes to ~/detections or the debug
image, and only on the newest frame - slower than the camera, older frames
are dropped.

Settings: config/visual_align.yaml, section sam_detector - read from the file
directly (no ROS parameters). Saved edits to prompts and conf apply within a
second; model, half, imgsz and image_topic only at start.

Started by control_launch.py (GPU PC, lerobot_jazzy env). On its own:
  ros2 run omniman_vla sam_detector.py
  ros2 run rqt_image_view rqt_image_view /sam_detector/debug/compressed
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
from ultralytics.models.sam import SAM3SemanticPredictor
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

SETTINGS = ['image_topic', 'model', 'prompts', 'conf', 'half', 'imgsz']

# Debug colours (BGR), cycled per prompt.
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


class SamDetector(Node):

    def __init__(self):
        super().__init__('sam_detector')
        # Every value comes from config/visual_align.yaml [sam_detector].
        self.cfg = Config('sam_detector', lambda v: [k for k in SETTINGS if k not in v],
                          self.get_logger())
        model = os.path.expanduser(self.cfg['model'])
        if not os.path.exists(model):
            raise RuntimeError(f'SAM 3 weights not found: {model} - see the top of this file')

        self.predictor = SAM3SemanticPredictor(overrides=dict(
            model=model, conf=float(self.cfg['conf']), half=bool(self.cfg['half']),
            imgsz=int(self.cfg['imgsz']), task='segment', mode='predict', save=False,
            verbose=False))
        # The first call builds the model and the CUDA kernels; do it now,
        # not on the first real frame.
        t = time.monotonic()
        self.detect(np.zeros((480, 640, 3), np.uint8), list(self.cfg['prompts']))
        self.get_logger().info(f'model ready in {time.monotonic() - t:.1f}s')

        self.det_pub = self.create_publisher(Detection2DArray, '~/detections', 10)
        self.debug_pub = self.create_publisher(CompressedImage, '~/debug/compressed', 1)
        self.create_subscription(
            CompressedImage, self.cfg['image_topic'], self.on_image, qos_profile_sensor_data)
        self.ms = []
        self.get_logger().info(f'ready - prompts in priority order: {self.cfg["prompts"]}')

    def detect(self, frame, prompts):
        """For each prompt, its best mask as (prompt, score, mask) - in prompt
        order, prompts not found left out."""
        self.predictor.args.conf = float(self.cfg['conf'])
        self.predictor.set_image(frame)
        result = self.predictor(text=prompts)[0]
        if result.masks is None or len(result.boxes) == 0:
            return []
        scores = result.boxes.conf.tolist()
        classes = [int(c) for c in result.boxes.cls.tolist()]
        masks = result.masks.data
        found = []
        for i, prompt in enumerate(prompts):
            hits = [j for j, c in enumerate(classes) if c == i]
            if hits:
                j = max(hits, key=lambda k: scores[k])
                found.append((prompt, scores[j], masks[j].cpu().numpy() > 0.5))
        return found

    def on_image(self, msg):
        want_det = self.det_pub.get_subscription_count() > 0
        want_debug = self.debug_pub.get_subscription_count() > 0
        if not (want_det or want_debug):
            return
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return

        t = time.monotonic()
        prompts = list(self.cfg['prompts'])
        found = self.detect(frame, prompts)
        ms = (time.monotonic() - t) * 1000.0
        self.ms.append(ms)
        if len(self.ms) >= 30:
            self.get_logger().info(f'inference {np.median(self.ms):.0f} ms median')
            self.ms = []

        out = Detection2DArray()
        out.header = msg.header
        drawn = []
        for prompt, score, mask in found:
            ys, xs = np.nonzero(mask)
            if len(xs) == 0:
                continue
            cx, cy = float(xs.mean()), float(ys.mean())
            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.size_x = float(xs.max() - xs.min())
            det.bbox.size_y = float(ys.max() - ys.min())
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = prompt
            hyp.hypothesis.score = float(score)
            det.results.append(hyp)
            out.detections.append(det)
            drawn.append((prompt, score, mask, cx, cy))
        self.det_pub.publish(out)

        if want_debug:
            for i, (prompt, score, mask, cx, cy) in enumerate(drawn):
                color = DEBUG_COLORS[prompts.index(prompt) % len(DEBUG_COLORS)]
                frame[mask] = (0.5 * frame[mask] + 0.5 * np.array(color)).astype(np.uint8)
                cv2.circle(frame, (int(cx), int(cy)), 5, color, -1)
                label = f'{i + 1}. {prompt} {score:.2f}' + ('  <- align' if i == 0 else '')
                cv2.putText(frame, label, (int(cx) - 40, max(int(cy) - 12, 16)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            cv2.putText(frame, f'{ms:.0f} ms', (8, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            dbg = CompressedImage()
            dbg.header = msg.header
            dbg.format = 'jpeg'
            dbg.data = cv2.imencode('.jpg', frame)[1].tobytes()
            self.debug_pub.publish(dbg)


def main():
    rclpy.init()
    node = SamDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
