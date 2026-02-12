#!/bin/bash

# Kill possible others running wrappers
pkill -f zed_wrapper
pkill -f throttle

echo "Starting with ROS ZED"

roslaunch zed_wrapper zedm.launch &
ZED_PID=$!
sleep 5

# IMU a 60 Hz
rosrun topic_tools throttle messages /zed/zed_node/imu/data 60.0 /zed/imu &
IMU_PID=$!

# Left and right 30 Hz
rosrun topic_tools throttle messages /zed/zed_node/left/image_rect_color 30.0 /zed/left &
LEFT_PID=$!
rosrun topic_tools throttle messages /zed/zed_node/right/image_rect_color 30.0 /zed/right &
RIGHT_PID=$!

# Camera info (for processing depth)
rosrun topic_tools relay /zed/zed_node/left/camera_info /zed/left_camera_info &
INFO_LEFT_PID=$!
rosrun topic_tools relay /zed/zed_node/right/camera_info /zed/right_camera_info &
INFO_RIGHT_PID=$!

echo "ROS nodes running"

trap "echo 'Stopping...';
kill $IMU_PID;
kill $LEFT_PID;
kill $RIGHT_PID;
kill $INFO_LEFT_PID;
kill $INFO_RIGHT_PID;
kill $ZED_PID;
exit 0" SIGINT

wait