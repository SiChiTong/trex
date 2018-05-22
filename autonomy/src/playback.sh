#!/bin/bash
xterm -e "roscore"
xterm -e "rosbag play ~/trex_loc_slow1_filter2.bag --clock"
