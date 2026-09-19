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

Tune with the debug image:
  ros2 run rqt_image_view rqt_image_view /color_detector/debug/compressed
"""

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# Settings every target needs, as <target>.<setting> in the params file.
TARGET_SETTINGS = ['hsv_low', 'hsv_high', 'min_area', 'min_fill', 'max_aspect']

# Debug colours (BGR), cycled per target.
DEBUG_COLORS = [(0, 0, 255), (0, 200, 0), (255, 0, 0), (0, 165, 255), (255, 0, 255)]


def declare_from_yaml(node, names):
    """Declare parameters with no default: every value must come from the
    params file. Any number type is accepted (300 or 300.0). Raises, naming
    the missing ones, if the file does not set them all."""
    for name in names:
        node.declare_parameter(name, descriptor=ParameterDescriptor(dynamic_typing=True))
    missing = [n for n in names if node.get_parameter(n).type_ == Parameter.Type.NOT_SET]
    if missing:
        raise RuntimeError(
            f'{node.get_name()}: not set in the params file: {", ".join(missing)} '
            '- run it with --params-file config/visual_align.yaml')


class ColorDetector(Node):

    def __init__(self):
        super().__init__('color_detector')
        # All values come from config/visual_align.yaml - none are set here.
        declare_from_yaml(self, ['image_topic', 'targets'])
        self.targets = list(self.get_parameter('targets').value)
        declare_from_yaml(self, [f'{t}.{s}' for t in self.targets for s in TARGET_SETTINGS])

        self.det_pub = self.create_publisher(Detection2DArray, '~/detections', 10)
        self.debug_pub = self.create_publisher(CompressedImage, '~/debug/compressed', 1)
        self.create_subscription(
            CompressedImage, self.get_parameter('image_topic').value,
            self.on_image, qos_profile_sensor_data)
        self.kernel = np.ones((5, 5), np.uint8)
        self.get_logger().info(f'ready - targets in priority order: {self.targets}')

    def p(self, target, setting):
        return self.get_parameter(f'{target}.{setting}').value

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
        for target in self.targets:
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
                color = DEBUG_COLORS[self.targets.index(target) % len(DEBUG_COLORS)]
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
