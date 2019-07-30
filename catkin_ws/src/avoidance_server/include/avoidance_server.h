//
// Active Intelligent Systems Laboratory
// Toyohashi University of Technology
//
// Yusuke Miake
//

#ifndef AVOIDANCE_SERVER_H_
#define AVOIDANCE_SERVER_H_

#include "avoidance_server/avoidance_server.h"

#include <ros/ros.h>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

namespace avoidance_server {
class AvoidanceServer {
  private:
    ros::NodeHandlePtr nh_;  // Node handle
    ros::Subscriber sub_;    // Subscriber for the clusters
    ros::ServiceServer srv_; // Move to destination
    tf2_ros::Buffer tf_buffer_;

  public:
    AvoidanceServer(ros::NodeHandlePtr node_handle);
    void subscriberCallback(const visualization_msgs::MarkerArrayConstPtr &clusters_msg);
    bool serviceCallback(avoidance_server::Request &req, avoidance_server::Response &res);
};
} // namespace avoidance_server

#endif // AVOIDANCE_SERVER_H_
