cmake_minimum_required(VERSION 2.8.3)
project(blade_nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
	FILES
  HitchPoseStamped.msg
	HitchPath.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)
catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable( HitchCommand src/HitchCommand.cpp src/KdTree.cpp)
target_link_libraries( HitchCommand ${catkin_LIBRARIES} )
