#include "viewpoint_evaluator/viewpoint_evaluator.h"

namespace viewpoint_evaluator_server {

/*-----------------------------
overview: Initialize a set of parameters and ROS services and subscribers
argument: ROS node handle
return: None
-----------------------------*/
ViewpointEvaluatorServer::ViewpointEvaluatorServer(ros::NodeHandlePtr node_handle) {
    nh_ = node_handle;
    setParam();
    get_nbv_srv_ = nh_->advertiseService("/viewpoint_planner_3d/get_next_viewpoint",
                                         &ViewpointEvaluatorServer::getNBV, this);
    get_candidates_cli_ = nh_->serviceClient<viewpoint_planner_3d::get_candidates>("/viewpoint_planner_3d/get_candidates");
    candidates_marker_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("/viewpoint_planner_3d/candidates_marker", 1);
    // Initialize Subscriber and Publisher
    odom_sub_ = nh_->subscribe(odom_topic, 1, &ViewpointEvaluatorServer::odomCallback, this);
    ROS_INFO("Ready to viewpoint_evaluator_server");
}

/*-----------------------------
overview: Processing at the end
argument: None
return: None
-----------------------------*/
ViewpointEvaluatorServer::~ViewpointEvaluatorServer() {
}

/*-----------------------------
overview: Set of ROS parameters
argument: None
Return: None
-----------------------------*/
void ViewpointEvaluatorServer::setParam() {
    ros::param::get("/viewpoint_evaluator/timeout", timeout);
    ros::param::get("/viewpoint_evaluator/candidate_marker_lifetime", candidate_marker_lifetime);
    ros::param::get("/viewpoint_evaluator/odom_topic", odom_topic);
}

/*-----------------------------
overview: Get current robot position
argument: None
Return: None
Set: odom
-----------------------------*/
void ViewpointEvaluatorServer::odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
    nav_msgs::Odometry odom = *odom_msg;
    current_robot_pose_ = odom.pose.pose;
}

/*-----------------------------
overview: Get next viewpoint(using ROS service)
argument: req, res (Take a look at get_next_viewpoint.srv)
return: is_succeeded - True if NBV (Next viewpoint) can be acquired normally
Ros searvice to use: get_viewpoint_candidates
-----------------------------*/
bool ViewpointEvaluatorServer::getNBV(viewpoint_planner_3d::get_next_viewpoint::Request &req,
                                      viewpoint_planner_3d::get_next_viewpoint::Response &res) {
    viewpoint_planner_3d::get_candidates srv;
    get_candidates_cli_.call(srv);
    if(srv.response.is_succeeded == true) {
        candidates = srv.response.candidates;
        ROS_INFO("Successful acquisition of viewpoint candidates");
    } else {
        ROS_ERROR("Failed to call get_candidates");
        res.is_succeeded = false;
        return false;
    }
    visualizationCandidates();
    geometry_msgs::Pose robot_pose = current_robot_pose_;
    std::cout << "robot_pose: " << robot_pose << std::endl;
    res.is_succeeded = true;
    return true;
}

/*-----------------------------
overview: Visualization of viewpoint candidates
argument: None
return: None
-----------------------------*/
void ViewpointEvaluatorServer::visualizationCandidates(void) {
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(candidates.size());
    int id = 0;
    for(geometry_msgs::Pose candidate : candidates) {
        visualization_msgs::Marker marker;
        marker_array.markers[id].header.frame_id = "/map";
        marker_array.markers[id].header.stamp = ros::Time::now();
        marker_array.markers[id].ns = "/candidate";
        marker_array.markers[id].id = id;
        marker_array.markers[id].type = visualization_msgs::Marker::ARROW;
        marker_array.markers[id].action = visualization_msgs::Marker::ADD;
        marker_array.markers[id].lifetime = ros::Duration(candidate_marker_lifetime);
        marker_array.markers[id].pose = candidate;
        marker_array.markers[id].scale.x = 0.1;
        marker_array.markers[id].scale.y = 0.01;
        marker_array.markers[id].scale.z = 0.01;
        marker_array.markers[id].color.r = 0.0f;
        marker_array.markers[id].color.g = 1.0f;
        marker_array.markers[id].color.b = 0.0f;
        marker_array.markers[id].color.a = 1.0f;
        id++;
    }
    candidates_marker_pub_.publish(marker_array);
}

} // namespace viewpoint_evaluator_server