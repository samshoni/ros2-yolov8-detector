# ROS2 YOLOv8 Detector (CPU) — Webcam → ROS Topics

Real-time object detection on **ROS2 Humble (Ubuntu 22.04)** using **YOLOv8 (CPU-only)**.

## What it publishes
- **Input:** `/image_raw` (`sensor_msgs/Image`) from `usb_cam`
- **Output:**
  - `/yolo/annotated` (`sensor_msgs/Image`) — bounding boxes + labels + FPS/latency overlay
  - `/yolo/summary` (`std_msgs/String`) — per-class counts (e.g., `person=1, bed=1 | total=2`)

## Run (one command)
```bash
./run_demo.sh
