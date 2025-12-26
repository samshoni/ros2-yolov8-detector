#!/usr/bin/env bash
set -e

WS="/media/sam/3add062a-b362-4679-bb69-aa05871474cb/projects/ml_ros2_ws"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# expose venv ML deps to ROS python
export PYTHONPATH="$WS/.venv/lib/python3.10/site-packages:$PYTHONPATH"

echo "[run_demo] Starting usb_cam..."
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 \
  -p image_width:=640 \
  -p image_height:=480 \
  -p pixel_format:=yuyv &
CAM_PID=$!

sleep 1

echo "[run_demo] Starting YOLO node..."
ros2 run yolo_ros2_detector yolo_node --ros-args -p image_topic:=/image_raw &
YOLO_PID=$!

# Give YOLO time to create publishers
sleep 2

echo
echo "[run_demo] Topics (should include /yolo/annotated and /yolo/summary):"
ros2 topic list | grep -E "image_raw|yolo" || true
echo
echo "[run_demo] Tip: Echo summary in another terminal:"
echo "  source /opt/ros/humble/setup.bash && ros2 topic echo /yolo/summary"
echo

echo "[run_demo] Trying to open rqt_image_view..."
set +e
ros2 run rqt_image_view rqt_image_view &
VIEW_PID=$!
set -e

cleanup() {
  echo
  echo "[run_demo] Stopping..."
  kill $YOLO_PID 2>/dev/null || true
  kill $CAM_PID 2>/dev/null || true
  kill $VIEW_PID 2>/dev/null || true
}
trap cleanup EXIT

wait

