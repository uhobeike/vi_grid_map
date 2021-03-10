cmake_minimum_required(VERSION 3.0.2)
project(vi_grid_map)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++14 -O3 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  vi_grid_map_msgs
  pluginlib
  rviz
)

find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED
  Core
  Widgets
)
set(QT_LIBRARIES Qt5::Widgets)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

set(SOURCE_FILES
  vigrid_cells_display.cpp
)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES
    rviz
  CATKIN_DEPENDS 
    geometry_msgs 
    std_msgs
    vi_grid_map_msgs
    pluginlib
)

###########
## Build ##
###########
set(CMAKE_AUTOMOC ON)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} 
  rviz_plugin/vigrid_cells_display.cpp
  rviz_plugin/vigrid_cells_display.h
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)


## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/test.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_pub_data_node src/test_pub_data.cpp)
target_link_libraries(${PROJECT_NAME}_pub_data_node ${catkin_LIBRARIES})

#############
## Install ##
#############
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install(DIRECTORY icons DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vi_grid_map.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
