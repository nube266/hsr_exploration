//
// Active Intelligent Systems Laboratory
// Toyohashi University of Technology
//
// Yusuke Miake
//

#include "./../include/avoidance_server.h"

namespace avoidance_server {

// Initialize service and subscriber
AvoidanceServer::AvoidanceServer(ros::NodeHandlePtr node_handle) {
    nh_ = node_handle;
    sub_ = nh_->subscribe<visualization_msgs::MarkerArray>("clusters", 1, &AvoidanceServer::subscriberCallback, this);
    srv_ = nh_->advertiseService("/avoidance_server", &AvoidanceServer::serviceCallback, this);
    ROS_INFO("Ready to avoidance_server");
}

// Subscribe point cloud(MarkerArrayConstPtr)
void AvoidanceServer::subscriberCallback(const visualization_msgs::MarkerArrayConstPtr &clusters_msg) {
    try {
        // Listen ot a transform from base_link to camera frame to fix the X-Y plane parallel to the floor
        transformStamped_ = tf_buffer_.lookupTransform("map", "head_rgbd_sensor_rgb_frame", ros::Time(0));
        transformStampedInverse_ = tf_buffer_.lookupTransform("head_rgbd_sensor_rgb_frame", "map", ros::Time(0));
    } catch(tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    tf::TransformListener listener;
    std::vector<cluster_tuple> cluster_tuples;
    for(auto cluster_it = clusters_msg->markers.begin(); cluster_it != clusters_msg->markers.end(); cluster_it++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_pc(new pcl::PointCloud<pcl::PointXYZ>);
        marker2PointCloud(*cluster_it, cluster_pc);

        Eigen::Vector4f centroid;
        Eigen::Vector3d centroid3d;

        pcl::compute3DCentroid(*cluster_pc, centroid);

        centroid3d << centroid[0], centroid[1], centroid[2];
        // Transform
        tf2::doTransform(centroid3d, centroid3d, transformStamped_);

        centroid[0] = centroid3d[0];
        centroid[1] = centroid3d[1];
        centroid[2] = centroid3d[2];

        // Check the thresholds
        setCentroidThreshold();
        if(centroid(0) < centroid_x_max_ || (centroid(1) < centroid_y_max_ || centroid(1) > centroid_y_min_) || (centroid(2) < centroid_z_max_ || centroid(2) > centroid_z_min_)) {
            std::cout << "cluster x:\t" << centroid(0) << "y:\t" << centroid(1) << std::endl;
        }
    }
}

// Move while avoiding small obstacles
bool AvoidanceServer::serviceCallback(avoidance_server::Request &req, avoidance_server::Response &res) {
    ROS_INFO("Move Start");
    res.is_succeeded = true;
    return true;
}

// Convert marker to PointCloud
void AvoidanceServer::marker2PointCloud(const visualization_msgs::Marker &input_marker, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_pc) {
    output_pc->header.frame_id = "head_rgbd_sensor_rgb_frame";
    for(auto point_it = input_marker.points.begin(); point_it != input_marker.points.end(); point_it++) {
        pcl::PointXYZ pcl_point;
        pcl_point.x = point_it->x;
        pcl_point.y = point_it->y;
        pcl_point.z = point_it->z;
        output_pc->push_back(pcl_point);
    }
}

void AvoidanceServer::setCentroidThreshold(void) {
    centroid_x_max_ = 1.5, centroid_y_max_ = 1.0, centroid_y_min_ = -1.0, centroid_z_max_ = 0.1, centroid_z_min_ = 0.0;
    nh_->getParam("/avoidance_server/centroid_x_max", centroid_x_max_);
    nh_->getParam("/avoidance_server/centroid_y_max", centroid_y_max_);
    nh_->getParam("/avoidance_server/centroid_y_min", centroid_y_min_);
    nh_->getParam("/avoidance_server/centroid_z_max", centroid_z_max_);
    nh_->getParam("/avoidance_server/centroid_z_min", centroid_z_min_);
}

} // namespace avoidance_server
