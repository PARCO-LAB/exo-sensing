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

# ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm camera_name:=zed_1 serial_number:=14938530 & # quella con supporto
# ZED_PID_1=$!
#ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm camera_name:=zed_2 serial_number:=18106099 & # quella senza supporto
# ZED_PID_2=$!
ros2 launch zed_multi_camera zed_multi_camera.launch.py cam_names:='[zed_1,zed_2]' cam_models:='[zedm,zedm]' cam_serials:='[18106099,14938530]' &
ZED_PID=$!

sleep 2

#ros2 bag record -o $BAG_DIR/$BAG_NAME /zed_2/zed_node/left/color/rect/image /zed/zed_node/imu/data /zed/zed_node/rgb/color/rect/camera_info /imu/P1 /imu/P2 /imu/P3 /imu/P4 imu/P5 /imu/P6 /zed_2/zed_node/depth/depth_registered &
ros2 bag record -o $BAG_DIR/$BAG_NAME  \
    /zed_multi/zed_1/left/color/rect/image/compressed /zed_multi/zed_2/left/color/rect/image/compressed \
    /zed_multi/zed_2/left/color/rect/camera_info /zed_multi/zed_1/left/color/rect/camera_info \
    /zed_multi/zed_1/right/color/rect/image/compressed /zed_multi/zed_2/right/color/rect/image/compressed \
    /zed_multi/zed_2/right/color/rect/camera_info /zed_multi/zed_1/right/color/rect/camera_info \
    /zed_multi/zed_1/depth/depth_registered /zed_multi/zed_2/depth/depth_registered \
    /zed_multi/zed_1/imu/data /zed_multi/zed_2/imu/data \
    /imu/P1 /imu/P2 /imu/P3 /imu/P4 imu/P5 /imu/P6 &
ROSBAG_PID=$!

echo "Recorder running"

trap "echo 'Stopping...';
kill $ROSBAG_PID;
kill $IMU_PID;
kill $ZED_PID;
exit 0" SIGINT

wait
