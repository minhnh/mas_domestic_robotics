cmake_minimum_required(VERSION 2.8.3)
project(mdr_object_recognition)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    roslint
    std_msgs
    sensor_msgs
    mas_perception_libs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
)

roslint_python()

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
