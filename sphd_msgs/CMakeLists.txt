cmake_minimum_required(VERSION 2.8.12)

project(sphd_msgs)

# message and service dependencies
set(MESSAGE_DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
  sensor_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   geometry_msgs
   message_generation
   ${MESSAGE_DEPENDENCIES}
)

add_message_files(
  DIRECTORY msg
  FILES
  SPHDMeasurement.msg
  SPHDMeasurements.msg
  SPHDMeasurementNew.msg
  SPHDMeasurementsNew.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  ${MESSAGE_DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  rospy 
  message_runtime 
  ${MESSAGE_DEPENDENCIES}
)

