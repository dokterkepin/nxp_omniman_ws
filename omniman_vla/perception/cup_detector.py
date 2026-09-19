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
  ros2 run omniman_vla cup_detector.py --ros-args \\
      --params-file src/omniman_vla/config/visual_align.yaml

Check what it sees:
  ros2 run rqt_image_view rqt_image_view /cup_detector/debug/compressed
"""

import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class CupDetector(Node):

    def __init__(self):
        super().__init__('cup_detector')
        self.declare_parameter('model', '~/models/yolov8s-worldv2.pt')
        # Text prompts - any of them counts as the target.
        self.declare_parameter('classes', ['paper cup', 'cup with yellow lid'])
        self.declare_parameter('image_topic', '/image_raw/compressed')
        self.declare_parameter('min_score', 0.3)
        self.declare_parameter('device', 'cuda:0')

        path = os.path.expanduser(self.get_parameter('model').value)
        classes = list(self.get_parameter('classes').value)
        self.min_score = float(self.get_parameter('min_score').value)
        self.device = self.get_parameter('device').value

        self.model = YOLO(path)
        if classes:
            self.model.set_classes(classes)
        # The first call is slow (CUDA init); do it now, not on the first frame.
        self.model(np.zeros((480, 640, 3), np.uint8), device=self.device, verbose=False)

        self.det_pub = self.create_publisher(Detection2DArray, '~/detections', 10)
        self.debug_pub = self.create_publisher(CompressedImage, '~/debug/compressed', 1)
        # Depth 1, best effort: a slow frame drops the ones behind it.
        self.create_subscription(
            CompressedImage, self.get_parameter('image_topic').value,
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
        result = self.model(frame, conf=self.min_score, device=self.device, verbose=False)[0]

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
