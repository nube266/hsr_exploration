//
// Active Intelligent Systems Laboratory
// Toyohashi University of Technology
//
// Yusuke Miake
//

#ifndef AVOIDANCE_SERVER_H_
#define AVOIDANCE_SERVER_H_

// my header
#include "avoidance_server/avoidance_server.h"

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// general
#include <string>
#include <vector>
#include <tuple>


namespace avoidance_server {
class AvoidanceServer {
  private:
    // ROS general
    ros::NodeHandlePtr nh_;  // Node handle
    ros::Subscriber sub_;    // Subscriber for the clusters
    ros::ServiceServer srv_; // Move to destination
    // TF
    tf2_ros::Buffer tf_buffer_;
    // geometry_msgs
    geometry_msgs::TransformStamped transformStamped_;
    geometry_msgs::TransformStamped transformStampedInverse_;

    // point_cloud
    std::tuple<int, char, std::string> t = std::make_tuple(1, 'a', "hello");
    // using cluster_tuple = std::tuple<Eigen::Vector4f, Eigen::Matrix3f, std::string, int, pcl::PointXYZ, pcl::PointCloud<pcl::PointXYZ>::Ptr>;
    // typedef std::tuple<Eigen::Vector4f, Eigen::Matrix3f, std::string, int, pcl::PointXYZ, pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_tuple;

  public:
    AvoidanceServer(ros::NodeHandlePtr node_handle);
    void subscriberCallback(const visualization_msgs::MarkerArrayConstPtr &clusters_msg);
    bool serviceCallback(avoidance_server::Request &req, avoidance_server::Response &res);
};
} // namespace avoidance_server

#endif // AVOIDANCE_SERVER_H_
