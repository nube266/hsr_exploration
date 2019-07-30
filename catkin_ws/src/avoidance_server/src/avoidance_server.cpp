//
// Active Intelligent Systems Laboratory
// Toyohashi University of Technology
//
// Yusuke Miake
//

#include "./../include/avoidance_server.h"

namespace avoidance_server {

AvoidanceServer::AvoidanceServer(ros::NodeHandlePtr node_handle) {
    nh_ = node_handle;
    // sub_ = nh_->subscribe<visualization_msgs::MarkerArray>("clusters", 1, &AvoidanceServer::subscriberCallback, this);
    srv_ = nh_->advertiseService("/avoidance_server", &AvoidanceServer::serviceCallback, this);
    ROS_INFO("Ready to avoidance_server");
}

void AvoidanceServer::subscriberCallback(const visualization_msgs::MarkerArrayConstPtr &clusters_msg) {
    /*
    try{
        // Listen ot a transform from base_link to camera frame to fix the X-Y plane parallel to the floor
        transformStamped_ = tf_buffer_.lookupTransform("base_link", "head_rgbd_sensor_rgb_frame", ros::Time(0));
        transformStampedInverse_ = tf_buffer_.lookupTransform("head_rgbd_sensor_rgb_frame", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    */
}

bool AvoidanceServer::serviceCallback(avoidance_server::Request &req, avoidance_server::Response &res) {
    ROS_INFO("Move Start");
    res.is_succeeded = true;
    return true;
}

} // namespace avoidance_server
