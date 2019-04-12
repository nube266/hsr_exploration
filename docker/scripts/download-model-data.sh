#!/bin/bash

################################################################################

# Download the YOLO pre-trained weights missing in the 'darknet_ros' package into the 'hsr_launch' package.
# http://pjreddie.com/media/files/yolov2.weights
# http://pjreddie.com/media/files/yolo9000.weights
# http://pjreddie.com/media/files/yolov3.weights
mkdir -p /root/HSR/catkin_ws/src/hsr_launch/config/darknet_ros/weights/
wget http://pjreddie.com/media/files/yolov2.weights -N -P /root/HSR/catkin_ws/src/hsr_launch/config/darknet_ros/weights
wget http://pjreddie.com/media/files/yolo9000.weights -N -P /root/HSR/catkin_ws/src/hsr_launch/config/darknet_ros/weights
wget http://pjreddie.com/media/files/yolov3.weights -N -P /root/HSR/catkin_ws/src/hsr_launch/config/darknet_ros/weights

# Link the downloaded file paths.
ln -s /root/HSR/catkin_ws/src/hsr_launch/config/darknet_ros/weights/* /root/HSR/catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights/
