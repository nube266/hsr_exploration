#!/bin/bash

################################################################################

# Run 'roslaunch' asynchronously in a subshell in the background.
roslaunch hsr_launch sdewg_gazebo_default.launch unattended:=true &

# Set simulation time.
sleep 600

# Get 'roslaunch' process ID and kill it.
kill $!
