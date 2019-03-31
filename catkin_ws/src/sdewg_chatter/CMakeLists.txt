cmake_minimum_required(VERSION 2.8.3)
project(sdewg_chatter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sdewg_talker src/sdewg_talker.cpp)
add_executable(sdewg_listener src/sdewg_listener.cpp)

target_link_libraries(sdewg_talker ${catkin_LIBRARIES})
target_link_libraries(sdewg_listener ${catkin_LIBRARIES})
