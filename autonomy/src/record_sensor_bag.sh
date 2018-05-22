#!/bin/bash
xterm -e "rosbag record --output-prefix=/home/ugv/param_study/unsorted /clock /cmd_vel /imu/data /odom /tf /usb_cam/image_raw /scan; bash"
