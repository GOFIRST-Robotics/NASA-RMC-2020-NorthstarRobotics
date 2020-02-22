cmake_minimum_required(VERSION 2.8.3)
project(rovr_navigation)
set(CMAKE_BUILD_TYPE Release)

file(GLOB SOURCES 
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11 -Wall -Wextra -Wno-switch)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  pcl_ros
)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS sensor_msgs roscpp
#  DEPENDS system_lib
  DEPENDS Threads
)

###########
## Build ##
###########

file(GLOB SOURCES 
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a C++ library
add_library(pc_filter
 ${SOURCES}
)
add_dependencies(pc_filter ${pc_filter_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(pc_filter_node src/pc_filter_node.cpp)
add_dependencies(pc_filter_node ${pc_filter_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pc_filter_node
   ${catkin_LIBRARIES}
   pc_filter
   ${PCL_LIBRARIES}
 )
