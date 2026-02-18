#!/bin/bash

# Path where to save the ROS bag
BAG_DIR=./ros_bags
DATE=$(date +"%Y-%m-%d_%H-%M-%S")
BAG_NAME="all_data_${DATE}.bag"

mkdir -p $BAG_DIR

source /opt/ros/humble/setup.bash
source /home/nvidia/exo-sensing/install/local_setup.bash
source ../.venv/bin/activate

hciconfig hci0 down && hciconfig hci0 up
python main.py &
IMU_PID=$!

ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm camera.camera_fps:=30 \
    depth.depth_mode:=NONE \
    depth.point_cloud_enabled:=false \
    positional_tracking.pos_tracking_enabled:=false \
    mapping.mapping_enabled:=false \
    object_detection.object_detection_enabled:=false \
    publish_tf:=false \
    sensors.publish_imu_tf:=false \
    sensors.sensors_pub_rate:=60 \
    camera.grab_resolution:=HD720 \
    camera.enable_image_enhancement:=false &
ZED_PID=$!

sleep 2

ros2 bag record -o $BAG_DIR/$BAG_NAME /zed/zed_node/rgb/color/rect/image /zed/zed_node/imu/data /zed/zed_node/rgb/color/rect/camera_info /imu/P1 /imu/P5 &
ROSBAG_PID=$!

echo "Recorder running"

trap "echo 'Stopping...';
kill $ROSBAG_PID;
kill $IMU_PID;
kill $ZED_PID;
exit 0" SIGINT

wait