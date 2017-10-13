cmake_minimum_required(VERSION 2.8.3)
project(mdr_perceive_plane_action)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  genmsg
  message_generation
  roslint
  rospy
)

catkin_python_setup()

add_action_files(
  FILES
  PerceivePlane.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

catkin_package(
 CATKIN_DEPENDS
   actionlib_msgs
   message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

roslint_python()
roslint_cpp()

