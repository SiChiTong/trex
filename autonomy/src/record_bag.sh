#!/bin/bash
xterm -e "rosbag record --split --size=1024 --output-prefix=unsorted jfr/robot/correction odometry/filtered target/camera_color target/camera_geom target/lidar usb_cam/image_raw scan jfr/target/position jfr/robot/entropy jfr/target/entropy; bash"
