cmake_minimum_required(VERSION 2.8.3)
project(sphd_sensor_models)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sphd_msgs
  rospy
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES phase3
#  CATKIN_DEPENDS geometry_msgs sphd_msgs rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

