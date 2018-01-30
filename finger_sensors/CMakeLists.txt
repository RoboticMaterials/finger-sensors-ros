cmake_minimum_required(VERSION 2.8.3)
project(finger_sensors)

find_package(catkin REQUIRED COMPONENTS
  finger_sensor_msgs
  roscpp
  rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

