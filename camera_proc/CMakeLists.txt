cmake_minimum_required(VERSION 2.8.3)
project(camera_proc)


find_package(catkin REQUIRED COMPONENTS
 roslaunch
 cv_bridge
 sensor_msgs
 roscpp
 image_transport
 std_msgs
 message_generation
tf2
tf2_ros
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(color_proc src/color_proc.cpp)
target_link_libraries(color_proc ${catkin_LIBRARIES})
target_link_libraries( color_proc ${OpenCV_LIBS} )

add_executable(color_proc_sim src/color_proc_sim.cpp)
target_link_libraries(color_proc_sim ${catkin_LIBRARIES})
target_link_libraries( color_proc_sim ${OpenCV_LIBS} )
