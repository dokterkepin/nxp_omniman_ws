#!/usr/bin/env python3
import os
import numpy as np
import cv2
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

from tf2_ros import Buffer, TransformListener, TransformException


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(BASE_DIR, "wood_best.pt")


class YoloTargetPublisher(Node):
    def __init__(self):
        super().__init__("yolo_target_publisher")

        self.declare_parameter("fx", 554.26)
        self.declare_parameter("fy", 554.26)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)
        self.declare_parameter("plane_z", 0.0)
        self.declare_parameter("camera_frame", "palm_link")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("conf", 0.25)

        self.fx = self.get_parameter("fx").value
        self.fy = self.get_parameter("fy").value
        self.cx = self.get_parameter("cx").value
        self.cy = self.get_parameter("cy").value
        self.plane_z = self.get_parameter("plane_z").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.conf = self.get_parameter("conf").value

        self.model = YOLO(MODEL_PATH)
        self.get_logger().info(f"Model loaded: {MODEL_PATH}")
        self.get_logger().info(f"Classes: {self.model.names}")

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            Image, "/image_raw", self._image_cb, qos_profile_sensor_data)

        self.pubs = {
            letter: self.create_publisher(PointStamped, f"/yolo_target/{letter}", 10)
            for letter in ["A", "F", "I", "R"]
        }

        self.window = "YOLO Targets"
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window, 640, 480)

    def _pixel_to_base(self, u, v):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.2))
        except TransformException as e:
            self.get_logger().warn(f"TF: {e}")
            return None

        rx = (u - self.cx) / self.fx
        ry = (v - self.cy) / self.fy
        rz = 1.0

        q = tf.transform.rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),     2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw),     1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw),     2 * (qy * qz + qx * qw),     1 - 2 * (qx * qx + qy * qy)],
        ])
        ray_base = R @ np.array([rx, ry, rz])

        ox = tf.transform.translation.x
        oy = tf.transform.translation.y
        oz = tf.transform.translation.z

        if abs(ray_base[2]) < 1e-6:
            return None
        t = (self.plane_z - oz) / ray_base[2]
        if t <= 0:
            return None
        x = ox + t * ray_base[0]
        y = oy + t * ray_base[1]
        return (x, y, self.plane_z)

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge: {e}")
            return

        results = self.model.predict(frame, conf=self.conf, iou=0.5,
                                     imgsz=640, verbose=False)
        dbg = frame.copy()

        if results and len(results[0].boxes) > 0:
            res = results[0]
            xyxy = res.boxes.xyxy.cpu().numpy()
            confs = res.boxes.conf.cpu().numpy()
            clses = res.boxes.cls.cpu().numpy().astype(int)

            for (x1, y1, x2, y2), conf, cls_id in zip(xyxy, confs, clses):
                letter = self.model.names.get(int(cls_id), str(int(cls_id)))
                u = float((x1 + x2) / 2.0)
                v = float((y1 + y2) / 2.0)

                pos = self._pixel_to_base(u, v)
                x1i, y1i, x2i, y2i = map(int, (x1, y1, x2, y2))
                cv2.rectangle(dbg, (x1i, y1i), (x2i, y2i), (0, 255, 0), 2)

                if pos is None:
                    cv2.putText(dbg, f"{letter} {conf:.2f} (no TF)",
                                (x1i, max(y1i - 6, 12)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    continue

                px, py, pz = pos
                pt = PointStamped()
                pt.header.stamp = msg.header.stamp
                pt.header.frame_id = self.base_frame
                pt.point.x = float(px)
                pt.point.y = float(py)
                pt.point.z = float(pz)

                if letter in self.pubs:
                    self.pubs[letter].publish(pt)

                cv2.putText(dbg,
                            f"{letter} {conf:.2f} ({px:.3f},{py:.3f})",
                            (x1i, max(y1i - 6, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                self.get_logger().info(
                    f"{letter} conf={conf:.2f} uv=({u:.1f},{v:.1f}) "
                    f"base=({px:.3f},{py:.3f},{pz:.3f})")
        else:
            cv2.putText(dbg, "no detections", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow(self.window, dbg)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = YoloTargetPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
