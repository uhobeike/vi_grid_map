cmake_minimum_required(VERSION 3.0.2)
project(vi_grid_map_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  actionlib_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  ViGridCells.msg
)

add_action_files(
  DIRECTORY action
  FILES
  ViGridMap.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs 
  actionlib_msgs
)
###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
    message_runtime 
    std_msgs
    geometry_msgs 
    actionlib_msgs
)
