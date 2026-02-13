#!/bin/bash

# Path where to save the ROS bag
BAG_DIR=./zed_bags
DATE=$(date +"%Y-%m-%d_%H-%M-%S")
BAG_NAME="zed_stereo_${DATE}.bag"

mkdir -p $BAG_DIR

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ./.venv/bin/activate

python main.py &
IMU_PID=$!

ros2 launch zed_wrapper zedm.launch.py \
    camera.camera_fps:=30 \
    depth.depth_mode:=NONE \
    depth.point_cloud_enabled:=false \
    positional_tracking.pos_tracking_enabled:=false \
    mapping.mapping_enabled:=false \
    object_detection.object_detection_enabled:=false \
    publish_tf:=false \
    sensors.publish_imu_tf:=false \
    camera.grab_resolution:=HD720 \
    camera.enable_image_enhancement:=false &
ZED_PID=$!

sleep 2

ros2 bag record -o $BAG_DIR/$BAG_NAME -a &
ROSBAG_PID=$!

echo "Recorder running"

trap "echo 'Stopping...';
kill $ROSBAG_PID;
kill $IMU_PID;
kill $ZED_PID;
exit 0" SIGINT

wait