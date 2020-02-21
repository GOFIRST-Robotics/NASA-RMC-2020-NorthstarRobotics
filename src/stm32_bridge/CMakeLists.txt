cmake_minimum_required(VERSION 2.8.3)
project(stm32_bridge)
set(CMAKE_BUILD_TYPE Release)

file(GLOB SOURCES 
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
        #"${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11 -Wall -Wextra -Wno-switch)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  socketcan_bridge
  can_msgs
  nav_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  ACHOOCmd.msg
  GESUNDHEITDoorCmd.msg
  GESUNDHEITExtendCmd.msg
  GESUNDHEITSpeedCmd.msg
  SNEEZEDigSpeedCmd.msg
  SNEEZETransSpeedCmd.msg
  SNEEZEHomeCmd.msg
  BLESSYOUPositionCmd.msg
  BLESSYOUSpeedCmd.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stm32_bridge
  CATKIN_DEPENDS can_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

file(GLOB SOURCES 
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
        #"${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(stm32_bridge
 ${SOURCES}
)
add_dependencies(stm32_bridge ${stm32_bridge_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(stm32_bridge_node src/stm32_bridge_node.cpp)
add_dependencies(stm32_bridge_node ${stm32_bridge_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stm32_bridge_node
   ${catkin_LIBRARIES}
   stm32_bridge
 )
