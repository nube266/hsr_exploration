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
  message_generation
  tf
  costmap_2d
  navfn
  image_transport
  cv_bridge
  visualization_msgs
)

find_package(
  OpenCV REQUIRED
)
find_package(
  octomap REQUIRED
)
find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_service_files(
  FILES
  generating_candidates.srv
  get_candidates.srv
  get_next_viewpoint.srv
  get_shortest_path_length.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  octomap_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
# LIBRARIES viewpoint_planner_src
  CATKIN_DEPENDS octomap_msgs octomap_ros roscpp rospy sensor_msgs std_msgs message_runtime image_transport cv_bridge visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(LINK_LIBS
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(generating_candidates
  src/generating_candidates/generating_candidates.cpp
  src/generating_candidates/generating_candidates_server.cpp
  ${SOURCE_FILES}
)
target_link_libraries(generating_candidates ${LINK_LIBS})
add_dependencies(generating_candidates ${viewpoint_planner_3d_EXPORTED_TARGETS})

add_executable(viewpoint_evaluator
  src/viewpoint_evaluator/viewpoint_evaluator.cpp
  src/viewpoint_evaluator/viewpoint_evaluator_server.cpp
  ${SOURCE_FILES}
)
target_link_libraries(viewpoint_evaluator ${LINK_LIBS})
add_dependencies(viewpoint_evaluator ${viewpoint_planner_3d_EXPORTED_TARGETS})