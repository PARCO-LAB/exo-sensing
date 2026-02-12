#!/bin/bash

# Path where to save the ROS bag
BAG_DIR=./zed_bags
DATE=$(date +"%Y-%m-%d_%H-%M-%S")
BAG_NAME="zed_stereo_${DATE}.bag"

mkdir -p $BAG_DIR

source ./.venv/bin/activate

python main.py &
IMU_PID=$!

chmod +x launch_zed.sh
launch_zed.sh &
ZED_PID=$!

sleep 2

rosbag record -O $BAG_DIR/$BAG_NAME -a &
ROSBAG_PID=$!

echo "Recorder running"

trap "echo 'Stopping...';
kill $ROSBAG_PID;
kill $IMU_PID;
kill $ZED_PID;
exit 0" SIGINT

wait