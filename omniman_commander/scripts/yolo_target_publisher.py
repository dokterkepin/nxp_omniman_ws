#!/usr/bin/env python3
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from ultralytics import YOLO


class YoloTargetPublisher(Node):
    def __init__(self):
        super().__init__('yolo_target_publisher')

        BASE_DIR = os.path.dirname(os.path.abspath(__file__))
        MODEL_PATH = os.path.join(BASE_DIR, 'wood_best.pt')
        self.model = YOLO(MODEL_PATH)

        self.sub_image = self.create_subscription(
            Image, '/image_raw', self.image_callback, qos_profile_sensor_data)

        self.pub_target_f = self.create_publisher(Point, '/yolo_target_f', 10)
        self.pub_target_i = self.create_publisher(Point, '/yolo_target_i', 10)
        self.pub_error_f = self.create_publisher(Point, '/yolo_error_f', 10)
        self.pub_error_i = self.create_publisher(Point, '/yolo_error_i', 10)

        self.get_logger().info('YOLO target publisher started, waiting for /image_raw ...')

    def image_callback(self, msg):
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        enc = msg.encoding.lower()
        if enc == 'bgr8':
            frame = raw.reshape(msg.height, msg.width, 3)
        elif enc == 'rgb8':
            frame = cv2.cvtColor(raw.reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2BGR)
        elif enc == 'mono8':
            frame = cv2.cvtColor(raw.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
        elif enc in ('yuyv', 'yuv422_yuy2'):
            frame = cv2.cvtColor(raw.reshape(msg.height, msg.width, 2), cv2.COLOR_YUV2BGR_YUYV)
        elif enc in ('uyvy', 'yuv422'):
            frame = cv2.cvtColor(raw.reshape(msg.height, msg.width, 2), cv2.COLOR_YUV2BGR_UYVY)
        else:
            self.get_logger().warn(f'Unsupported encoding: {msg.encoding}, trying raw reshape')
            frame = raw.reshape(msg.height, msg.width, -1)
            if frame.shape[2] != 3:
                return

        frame_cx = frame.shape[1] / 2.0
        frame_cy = frame.shape[0] / 2.0

        results = self.model.predict(source=frame, conf=0.5, verbose=False)

        if results and len(results[0].boxes) > 0:
            for box in results[0].boxes:
                label = self.model.names[int(box.cls[0])]
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0

                # Draw bounding box and label
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(frame, f'{label} ({int(cx)},{int(cy)})',
                            (int(x1), int(y1) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                pt = Point()
                pt.x = cx
                pt.y = cy
                pt.z = 0.0

                err = Point()
                err.x = cx - frame_cx
                err.y = cy - frame_cy
                err.z = 0.0

                if label == 'F':
                    self.pub_target_f.publish(pt)
                    self.pub_error_f.publish(err)
                    self.get_logger().info(f'F detected at ({cx:.1f}, {cy:.1f}) err=({err.x:+.1f}, {err.y:+.1f})')
                elif label == 'I':
                    self.pub_target_i.publish(pt)
                    self.pub_error_i.publish(err)
                    self.get_logger().info(f'I detected at ({cx:.1f}, {cy:.1f}) err=({err.x:+.1f}, {err.y:+.1f})')

        # Always refresh the display and pump the GUI event loop,
        # even when nothing is detected — otherwise the window freezes.
        cv2.imshow('YOLO Target', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloTargetPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
