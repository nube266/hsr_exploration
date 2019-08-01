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
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

// PCL
#include <pcl/common/pca.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// general
#include <iostream>
#include <list>
#include <string>
#include <vector>

namespace avoidance_server {
class AvoidanceServer {
  private:
    // ROS general
    ros::NodeHandlePtr nh_;  // Node handle
    ros::Subscriber sub_;    // Subscriber for the clusters
    ros::ServiceServer srv_; // Move to destination
    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener *tf_listener_;
    // geometry_msgs
    geometry_msgs::TransformStamped transformStamped_;
    geometry_msgs::TransformStamped transformStampedInverse_;
    // point_cloud
    using cluster_tuple = std::tuple<Eigen::Vector4f, Eigen::Matrix3f, std::string, int,
                                     pcl::PointXYZ, pcl::PointCloud<pcl::PointXYZ>::Ptr>;
    double centroid_x_max_; // CentroidThreshold
    double centroid_y_max_; //     |
    double centroid_y_min_; //     |
    double centroid_z_max_; //     |
    double centroid_z_min_; //     |
    // for path_server
    struct Point {
        double x;
        double y;
    };
    std::vector<Point> obstacle_points_;

    // Convert marker to PointCloud
    void marker2PointCloud(const visualization_msgs::Marker &input_marker,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &output_pc);
    // Set the threshold using getParam
    void setCentroidThreshold(void);
    // Set Obstacle points for path_server
    void setObstaclePoints(const visualization_msgs::MarkerArrayConstPtr &clusters_msg);

  public:
    // Initialize service and subscriber
    AvoidanceServer(ros::NodeHandlePtr node_handle);
    // Destructor
    ~AvoidanceServer();
    // Subscribe point cloud(MarkerArrayConstPtr)
    void subscriberCallback(const visualization_msgs::MarkerArrayConstPtr &clusters_msg);
    // Get obstacle points
    bool serviceCallback(avoidance_server::Request &req, avoidance_server::Response &res);
};
} // namespace avoidance_server

#endif // AVOIDANCE_SERVER_H_
