cmake_minimum_required(VERSION 3.0.2)
project(vi_grid_map_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
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

generate_messages(DEPENDENCIES geometry_msgs std_msgs)
###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
    geometry_msgs 
    message_runtime 
    std_msgs
)
