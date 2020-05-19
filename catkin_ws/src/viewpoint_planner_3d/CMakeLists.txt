cmake_minimum_required(VERSION 2.8.3)
project(viewpoint_planner_3d)

# C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo

find_package(catkin REQUIRED COMPONENTS
  octomap_msgs
  octomap_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(octomap REQUIRED)

add_service_files(
  FILES
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  octomap_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  octomap_msgs
  octomap_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)
