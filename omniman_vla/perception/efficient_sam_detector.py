#!/usr/bin/env python3
"""
Finds align targets with EfficientSAM3 - a distilled, lighter SAM 3 - prompted
with text. A drop-in alternative to sam_detector.py: same output, same debug
image, same behaviour; only the model differs.

    in   /image_raw/compressed            sensor_msgs/CompressedImage
    out  ~/detections                     vision_msgs/Detection2DArray
         ~/debug/compressed               sensor_msgs/CompressedImage   masks drawn

    in   /visual_align/target             std_msgs/String   latched: the prompt

The prompt is the target the mission's Align sets (/visual_align/target),
so any text written in a mission just works; until one is set, nothing is
detected. One detection when it is found (its highest-scoring mask),
class_id = the prompt. The position is the mask's centroid.

Measured on the recorded frames against SAM 3 (RTX 4060 Ti, bf16, 1008 px):
~65 ms a frame, ~1.1 GB of GPU memory (SAM 3: 3.4 GB). RV-M (RepViT) with
"yellow cup lid" found the cup in 20/20 aligned frames, like SAM 3. The flat
black mark is weak: at best 5/7 ("black rectangle", conf 0.3-0.4), with
frames that have no mark scoring up to ~0.5 - expect misses and false hits.
Other wordings ("paper cup", "cup", "black sticker") mostly fail.

EfficientSAM3 is not in Ultralytics: it runs from its own code in the
omniman_vla conda env (github.com/SimonZeng7108/efficientsam3, installed with
`pip install -e ~/tools/efficientsam3/sam3` - see docs/omniman_vla.md
"Setup"), weights from
huggingface.co/Simon7108528/EfficientSAM3 (efficientsam3_ft/*.pt). The
released checkpoints use the MobileCLIP-S0 text encoder, context 16 - with
S1 the text weights do not load and nothing is ever detected.

Settings: config/visual_align.yaml, section efficient_sam_detector - read from
the file directly (no ROS parameters). Saved edits to conf apply
within a second; the model settings and image_topic only at start.

Started by control_launch.py in place of sam_detector (omniman_vla env). On
its own:
  conda activate omniman_vla && source install/setup.bash
  ros2 run omniman_vla efficient_sam_detector.py
"""

import os
import time

import cv2
import numpy as np
import rclpy
import torch
import yaml
from ament_index_python.packages import get_package_share_directory
from PIL import Image
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sam3.model.sam3_image_processor import Sam3Processor
from sam3.model_builder import build_efficientsam3_image_model
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

SETTINGS = ['image_topic', 'model', 'backbone_type', 'model_name', 'text_encoder',
            'text_context_length', 'resolution', 'conf']

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


class EfficientSamDetector(Node):

    def __init__(self):
        super().__init__('efficient_sam_detector')
        # Every value comes from config/visual_align.yaml [efficient_sam_detector].
        self.cfg = Config('efficient_sam_detector', lambda v: [k for k in SETTINGS if k not in v],
                          self.get_logger())
        model = os.path.expanduser(self.cfg['model'])
        if not os.path.exists(model):
            raise RuntimeError(f'EfficientSAM3 weights not found: {model} - '
                               'see the top of this file')

        net = build_efficientsam3_image_model(
            checkpoint_path=model, backbone_type=self.cfg['backbone_type'],
            model_name=str(self.cfg['model_name']), text_encoder_type=self.cfg['text_encoder'],
            text_encoder_context_length=int(self.cfg['text_context_length']),
            load_from_HF=False)
        self.processor = Sam3Processor(net, resolution=int(self.cfg['resolution']),
                                       confidence_threshold=float(self.cfg['conf']))
        # The first call builds the model and the CUDA kernels; do it now,
        # not on the first real frame.
        t = time.monotonic()
        # Any word - warms up the text path too.
        self.detect(np.zeros((480, 640, 3), np.uint8), ['object'])
        self.get_logger().info(f'model ready in {time.monotonic() - t:.1f}s')

        self.det_pub = self.create_publisher(Detection2DArray, '~/detections', 10)
        self.debug_pub = self.create_publisher(CompressedImage, '~/debug/compressed', 1)
        self.create_subscription(
            CompressedImage, self.cfg['image_topic'], self.on_image, qos_profile_sensor_data)
        # The mission's Align sets what to look for (/visual_align/target,
        # latched); nothing is detected until it is set.
        self.target = ''
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/visual_align/target', self.on_target, latched)
        self.ms = []
        self.get_logger().info('ready - waiting for a target on /visual_align/target')

    def on_target(self, msg):
        self.target = msg.data.strip()
        self.get_logger().info(f'looking for: "{self.target}"')

    @torch.inference_mode()
    def detect(self, frame, prompts):
        """For each prompt, its best mask as (prompt, score, mask) - in prompt
        order, prompts not found left out. The image is encoded once; each
        prompt then runs only the (small) text and decoder part."""
        self.processor.confidence_threshold = float(self.cfg['conf'])
        found = []
        with torch.autocast('cuda', dtype=torch.bfloat16):
            state = self.processor.set_image(Image.fromarray(frame[:, :, ::-1]))
            for prompt in prompts:
                state = self.processor.set_text_prompt(prompt, state)
                scores = state.get('scores')
                if scores is None or len(scores) == 0:
                    continue
                j = int(torch.argmax(scores))
                mask = state['masks'][j].squeeze().float().cpu().numpy() > 0.5
                found.append((prompt, float(scores[j]), mask))
        return found

    def on_image(self, msg):
        want_det = self.det_pub.get_subscription_count() > 0
        want_debug = self.debug_pub.get_subscription_count() > 0
        if not self.target or not (want_det or want_debug):
            return
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return

        t = time.monotonic()
        prompts = [self.target]
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
    node = EfficientSamDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
