cmake_minimum_required(VERSION 3.0.2)
project(raspicat_speak)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++14 -O3 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  topic_tools
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

###########
## Build ##
###########
add_library(raspicat_speak
  src/japanese_speak.cpp
)
add_executable(raspicat_speak_node
  src/japanese_speak_node.cpp
)
target_link_libraries(
  raspicat_speak_node
  raspicat_speak
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############