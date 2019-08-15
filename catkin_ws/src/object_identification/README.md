# What is this package?
It is ROS package for object regisitration and recognition.

# ROS parameter
 these parameter can set lanch file.
 1. point_topic

    this topic use for detect object.

 2. image_topic

    This topic use for calcucate histogram.
 
 3. camerainfo_topic
 
    This topic use for calcucate histogram.

# Notes
 - This ROS node write files to "database" directory if you call "/object_identification/register_object" service, and it doesn’t erase these files. 
 So if you want to delete the registered data, please delete the file manually before launching ROS nodes.
 - When you use YCBdataset, you change the name of the direcory from YCBdataset to dataset. 

# Node API
## object_identification
### subscribe topic
 - point_topic (sensor_msgs::PointCloud2)
 - image_topic (sensor_msgs::Image)
 - camerainfo_topic (sensor_msgs::CameraInfo)
### publsh topic
 - recongnition_result (darknet_ros_msgs::BoundingBoxes)
### service
 - /object_identification/register_object

 This service clusters colored point clouds and treats the closest cluster as the registration target. 
 And it calculate color histograms of target for recognition and save as file with name given by augment.
 This service return attributes with score.
 
 ”cluster” in this program indicates box filter, remove plane, euclidean  clustering.
 
 - /object_identification/training

 This service read saved color histogram datas and store memory. If morohashi-san’s stady was usable, I wanted to train the model abot SVM or random forest for recognition. But, it is not implemented.
 
 - /object_identification/recognition_object
 
This service clusters colored point clouds and compare the each clusters’ color histogram and stored color histogram, and answer the name of the object with the  highest similarity.
 

## attribute_estimate_node

### subscribe topic
   None 
### publsh topic
   None
### service
 - /get_attribute

This service estimate attributes from image.This service return attributes with score.


