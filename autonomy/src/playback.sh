#!/bin/bash
xterm -e "roscore"
xterm -e "rosbag play /home/ugv/trex_loc_slow1_filter2.bag --clock"
