# 起動時のメモ
gazeboとrvizの起動時  

```
roslaunch hsr_launch sdewg_gazebo_default.launch  
roslaunch octomap_publisher octomap_publisher.launch  
roslaunch hsr_launch flexbe_app_default.launch  
roslaunch viewpoint_planner_3d viewpoint_planner_3d.launch
```

## darknet_rosについて
darknet_rosはgit cloneした段階だと重みファイルがない。  
別途ダウンロードして以下に配置すること。  
/home/aisl/Program/ROS/hsr_exploration/catkin_ws/src/hsr_launch/config/darknet_ros/weights/yolov3.weights  
