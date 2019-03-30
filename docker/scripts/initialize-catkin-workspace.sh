#!/bin/bash

################################################################################

# Download package lists from Ubuntu repositories.
apt-get update

# Install system dependencies required by specific ROS packages.
# http://wiki.ros.org/rosdep
rosdep update

# Source the updated ROS environment.
source /opt/ros/kinetic/setup.bash

################################################################################

# Initialize and build the Catkin workspace.
cd /root/HSR/catkin_ws/ && catkin_make -DCMAKE_BUILD_TYPE=Release

# Source the Catkin workspace.
source /root/HSR/catkin_ws/devel/setup.bash
