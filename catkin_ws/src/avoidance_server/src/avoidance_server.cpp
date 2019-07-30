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

    tf::TransformListener listener;
    // std::vector<cluster_tuple> cluster_tuples;
    // for (auto cluster_it = clusters_msg->markers.begin(); cluster_it != clusters_msg->markers.end(); cluster_it++) {
        /*
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
        if(centroid(0) > centroid_x_max_ ||
        (centroid(1) > centroid_y_max_ || centroid(1) < centroid_y_min_) || 
        (centroid(2) > centroid_z_max_ || centroid(2) < centroid_z_min_)) {
        ROS_WARN("Not within the threshold : %f %f %f", centroid(0), centroid(1), centroid(2));
        ROS_WARN("               threshold : %f %f %f %f %f", centroid_x_max_, centroid_y_min_, centroid_y_max_, centroid_z_min_, centroid_z_max_);

        continue;
        }

        pcl::PointXYZ near = getNearest(cluster_pc);
        pcl::PCA<pcl::PointXYZ> pca = pcl::PCA<pcl::PointXYZ>(true);
        pca.setInputCloud(cluster_pc);
        // Vector in Camera frame
        Eigen::Matrix3f basis_vector = pca.getEigenVectors();

        // Make a tuple
        cluster_tuple tpl(centroid, basis_vector, cluster_it->header.frame_id, cluster_it->id, near, cluster_pc);

        // Store the tuple
        cluster_tuples.push_back(tpl);
        */
    // }
}

bool AvoidanceServer::serviceCallback(avoidance_server::Request &req, avoidance_server::Response &res) {
    ROS_INFO("Move Start");
    res.is_succeeded = true;
    return true;
}

} // namespace avoidance_server
