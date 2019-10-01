cmake_minimum_required(VERSION 2.8.3)
project(rosweld_tools)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
  visualization_msgs
)

add_service_files(FILES
  SetService.srv
)

add_message_files(
   DIRECTORY msg
   FILES
   Calibration.msg
   CalibrationPoint.msg
   Project.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   visualization_msgs
 )

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs visualization_msgs
)

