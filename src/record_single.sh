#!/bin/bash

# Path where to save the ROS bag
BAG_DIR=/mnt/nvme/ros_bags
#BAG_DIR=./ros_bags
DATE=$(date +"%Y-%m-%d_%H-%M-%S")
BAG_NAME="all_data_${DATE}.bag"

mkdir -p $BAG_DIR

source /opt/ros/humble/setup.bash
source /home/nvidia/exo-sensing/install/local_setup.bash
source ../.venv/bin/activate

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

hciconfig hci0 down && hciconfig hci0 up
python main.py &
IMU_PID=$!

ros2 launch realsense2_camera rs_launch.py config_file:='./config_d455.yaml' &
D455_PID=$!

# ros2 launch realsense2_camera rs_multi_camera_launch.py config_file1:=config_d455_1.yaml config_file2:=config_d455_2.yaml &
# D455_PID=$!

sleep 2



ros2 bag record -o $BAG_DIR/$BAG_NAME \
    /camera/camera/color/image_raw/compressed /camera/camera/aligned_depth_to_color/image_raw /camera/camera/imu\
    /camera/camera/color/camera_info /camera/camera/aligned_depth_to_color/camera_info /camera/camera/color/metadata /camera/camera/depth/metadata \
    /camera/camera/accel/imu_info /camera/camera/gyro/imu_info /camera/camera/accel/metadata /camera/camera/gyro/metadata /camera/camera/extrinsics/depth_to_color \
    /imu/P1 /imu/P2 /imu/P3 /imu/P4 imu/P5 /imu/P6 /imu/G /imu/H /imu/M &
ROSBAG_PID=$!

echo "Recorder running"

trap "echo 'Stopping...';
kill $ROSBAG_PID;
kill $IMU_PID;
kill $D455_PID;
exit 0" SIGINT

wait
