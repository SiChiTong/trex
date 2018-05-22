#!/bin/bash
xterm -e "rosbag record --output-prefix=unsorted /clock /cmd_vel /imu/data /odom /tf /usb_cam/image_raw /scan; bash"
