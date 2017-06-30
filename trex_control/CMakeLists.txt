cmake_minimum_required(VERSION 2.8.3)
project(trex_control)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch/trex_control.launch)
roslaunch_add_file_check(launch/teleop.launch)
roslaunch_add_file_check(launch/trex_hw.launch)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS scripts/stow_ur5.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
