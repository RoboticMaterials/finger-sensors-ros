cmake_minimum_required(VERSION 2.8.3)
project(finger_sensor_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib
  actionlib_msgs
  geometry_msgs
)

add_message_files(
  FILES
  FingerTouch.msg
  FingerFAI.msg
  FingerSAI.msg
)


generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs geometry_msgs message_runtime
)

