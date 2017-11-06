cmake_minimum_required(VERSION 2.8.3)
project(mdr_lwr_kinematics)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf_conversions
  rospy
  geometry_msgs
  actionlib_msgs
  actionlib
)

include_directories(
  include
  #SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
  actionlib_msgs
)

set(IKFAST_LIBRARY_NAME lwr_moveit_ikfast_plugin)

find_package(LAPACK REQUIRED)

catkin_python_setup()

add_library(${IKFAST_LIBRARY_NAME}
  src/lwr_ikfast_moveit_plugin.cpp
  src/solution_comparator.cpp
)
target_link_libraries(${IKFAST_LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

install(TARGETS ${IKFAST_LIBRARY_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(
  FILES
    lwr_moveit_ikfast_plugin_description.xml
  DESTINATION
     ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
