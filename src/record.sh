#!/bin/bash

# Path where to save the ROS bag
BAG_DIR=/mnt/nvme/ros_bags
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


# ros2 launch zed_multi_camera zed_multi_camera.launch.py cam_names:='[zed_1,zed_2]' cam_models:='[zedm,zedm]' cam_serials:='[18106099,14938530]' &
# ZED_PID=$!

# ros2 launch realsense2_camera rs_dual_camera_launch.py camera_name1:='cam1' camera_name2:='cam2' config_file1:='config_d455_1.yaml' config_file2:='config_d455_2.yaml'
ros2 launch realsense2_camera rs_launch.py config_file:='./config_d455.yaml' &
# ros2 launch realsense2_camera rs_multi_camera_launch.py \
#   camera_name1:=camera1 serial_no1:=_339522300582 \
#   camera_name2:=camera2 serial_no2:=_243122302200 \
#   depth_module.depth_profile1:=1280x720x30 \
#   rgb_camera.profile1:=1280x720x30 \
#   depth_module.depth_profile2:=1280x720x30 \
#   rgb_camera.profile2:=1280x720x30 \
#   gyro_fps1:=200 accel_fps1:=100 unite_imu_method1:=2 \
#   gyro_fps2:=200 accel_fps2:=100 unite_imu_method2:=2 &

D455_PID=$!

sleep 10

# ros2 bag record -o $BAG_DIR/$BAG_NAME  \
#     /zed_multi/zed_1/left/color/rect/image/compressed /zed_multi/zed_2/left/color/rect/image/compressed \
#     /zed_multi/zed_2/left/color/rect/camera_info /zed_multi/zed_1/left/color/rect/camera_info \
#     /zed_multi/zed_1/right/color/rect/image/compressed /zed_multi/zed_2/right/color/rect/image/compressed \
#     /zed_multi/zed_2/right/color/rect/camera_info /zed_multi/zed_1/right/color/rect/camera_info \
#     /zed_multi/zed_1/depth/depth_registered /zed_multi/zed_2/depth/depth_registered \
#     /zed_multi/zed_1/imu/data /zed_multi/zed_2/imu/data \
#     /imu/P1 /imu/P2 /imu/P3 /imu/P4 imu/P5 /imu/P6 &

ros2 bag record -o $BAG_DIR/$BAG_NAME \
    /camera/camera/color/image_raw/compressed /camera/camera/depth/image_rect_raw /camera/camera/imu \
    /camera/camera/color/camera_info /camera/camera/depth/camera_info /camera/camera/color/metadata /camera/camera/depth/metadata \
    /camera/camera/accel/imu_info /camera/camera/gyro/imu_info /camera/camera/accel/metadata /camera/camera/gyro/metadata /camera/camera/extrinsics/depth_to_color\
    /imu/P1 /imu/P2 /imu/P3 /imu/P4 imu/P5 /imu/P6 &
ROSBAG_PID=$!

echo "Recorder running"

trap "echo 'Stopping...';
kill $ROSBAG_PID;
kill $IMU_PID;
kill $D455_PID;
exit 0" SIGINT

wait
