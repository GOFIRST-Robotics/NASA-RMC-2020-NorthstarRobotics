cmake_minimum_required(VERSION 2.8.3)
project(rovr_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -g -O0)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_manager
  robot_state_publisher
  joint_state_publisher
  joint_state_controller
  sensor_msgs
  geometry_msgs
  can_msgs
  telecom
  formatter_string
  socketcan_bridge
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rovr_control
  CATKIN_DEPENDS
    roscpp
    controller_manager
    robot_state_publisher
    joint_state_publisher
    joint_state_controller
    sensor_msgs
    geometry_msgs
    can_msgs
    socketcan_bridge
    # include our pkgs
    telecom
    formatter_string
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(teleop_twist_node src/teleop_twist_node.cpp)
target_link_libraries(teleop_twist_node ${catkin_LIBRARIES})

add_executable(robot_to_mc_node src/telecom_nodes/robot_to_mc_node.cpp)
target_link_libraries(robot_to_mc_node
  telecom
  formatter_string
  ${catkin_LIBRARIES}
)
add_executable(mc_to_robot_node src/telecom_nodes/mc_to_robot_node.cpp)
target_link_libraries(mc_to_robot_node
  telecom
  formatter_string
  ${catkin_LIBRARIES}
)

add_executable(joy_to_onboard_direct_node src/joy_to_onboard_direct_node.cpp)
target_link_libraries(joy_to_onboard_direct_node ${catkin_LIBRARIES})
