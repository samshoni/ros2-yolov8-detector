#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
from ultralytics import YOLO


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_detector_node")

        # Parameters
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("conf", 0.35)
        self.declare_parameter("imgsz", 640)

        self.image_topic = self.get_parameter("image_topic").value
        self.model_name = self.get_parameter("model").value
        self.conf = float(self.get_parameter("conf").value)
        self.imgsz = int(self.get_parameter("imgsz").value)

        self.bridge = CvBridge()

        # Load model
        self.get_logger().info(f"Loading YOLO model: {self.model_name} (CPU)")
        self.model = YOLO(self.model_name)

        # ROS I/O
        self.sub = self.create_subscription(Image, self.image_topic, self.cb_image, 10)
        self.pub_annot = self.create_publisher(Image, "/yolo/annotated", 10)
        self.pub_summary = self.create_publisher(String, "/yolo/summary", 10)

        # FPS tracking
        self.last_t = time.time()
        self.fps_smooth = 0.0

        self.get_logger().info(f"Subscribed to: {self.image_topic}")
        self.get_logger().info("Publishing annotated images to: /yolo/annotated")
        self.get_logger().info("Publishing summary text to: /yolo/summary")

    def cb_image(self, msg: Image):
        t0 = time.time()

        # ROS Image -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Inference
        results = self.model.predict(
            source=frame,
            conf=self.conf,
            imgsz=self.imgsz,
            verbose=False,
            device="cpu"
        )

        # Build a summary: counts per class
        names = results[0].names  # class_id -> name
        counts = {}
        for b in results[0].boxes:
            cls_id = int(b.cls.item())
            name = names.get(cls_id, str(cls_id))
            counts[name] = counts.get(name, 0) + 1

        if counts:
            parts = [f"{k}={v}" for k, v in sorted(counts.items(), key=lambda x: (-x[1], x[0]))]
            summary_text = ", ".join(parts)
            total = sum(counts.values())
        else:
            summary_text = "no detections"
            total = 0

        summary = String()
        summary.data = f"{summary_text} | total={total}"
        self.pub_summary.publish(summary)

        # Annotated image
        annotated = results[0].plot()  # BGR image with boxes/labels

        # FPS overlay
        dt = time.time() - self.last_t
        self.last_t = time.time()
        fps = (1.0 / dt) if dt > 0 else 0.0
        self.fps_smooth = 0.9 * self.fps_smooth + 0.1 * fps if self.fps_smooth > 0 else fps

        latency_ms = (time.time() - t0) * 1000.0
        cv2.putText(
            annotated,
            f"FPS: {self.fps_smooth:.1f} | Latency: {latency_ms:.1f} ms",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        out = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out.header = msg.header
        self.pub_annot.publish(out)


def main():
    rclpy.init()
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

