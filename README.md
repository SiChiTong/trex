TREX
====

This contains all files need to build, run, and simulate the Virginia Tech unmanned ground vehicle, TREX.

 - trex_control : Control configuration for the TREX
 - trex_description : Robot description for both the real robot and simulation
 - trex_gazebo : Simulation demo for the TREX
 - trex_maps : Pre-built maps for use in TREX simulation
 - trex_navigation : Navigation configurations and demos
 - trex_sawyer_moveit_config : MoveIt configurations and demos using the Rethink Robotic arm Sawyer
 - trex_ur5_moveit : MoveIt configurations and demos using the Universal Robotics arm UR5

To use these packages, install the following into your catkin workspace and catkin make

 - cd ~/catkin_ws/src/
 - git clone https://github.com/johodges/trex
 - git clone https://github.com/johodges/sawyer_gazebo
 - git clone https://github.com/RethinkRobotics/sawyer_robot
 - git clone https://github.com/RethinkRobotics/intera_sdk
 - git clone https://github.com/RethinkRobotics/intera_common
 - git clone https://github.com/RethinkRobotics/sawyer_moveit
 - cd ~/catkin_ws && catkin_make

To run the TREX by itself

 - roslaunch trex_gazebo playpen_trex.launch

To run the TREX with the UR5 arm

 - roslaunch trex_gazebo playpen_ur5.launch

To run the TREX with the Sawyer arm

 - roslaunch trex_gazebo playpen_sawyer.launch
