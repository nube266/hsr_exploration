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
    get_nbv_srv_ = nh_->advertiseService("/viewpoint_planner_3d/get_next_viewpoint", &ViewpointEvaluatorServer::getNBV, this);
    get_candidates_cli_ = nh_->serviceClient<viewpoint_planner_3d::get_candidates>("/viewpoint_planner_3d/get_candidates");
    candidates_marker_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("/viewpoint_planner_3d/candidates_marker", 1);
    make_path_cli_ = nh_->serviceClient<nav_msgs::GetPlan>("/move_base/make_plan", true);
    clear_costmaps_cli_ = nh_->serviceClient<std_srvs::Empty>("/move_base/clear_costmaps", true);
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
    ros::param::get("/viewpoint_evaluator/path_planning_tolerance", path_planning_tolerance);
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
overview: Returns the total travel distance in the entered travel plan
argument: plan(toravel plan)
Return: distance[m]
-----------------------------*/
double ViewpointEvaluatorServer::calcTravelDistance(const nav_msgs::Path &plan) {
    double distance = 0.0;
    for(size_t i = 0; i < plan.poses.size() - 1; ++i) {
        geometry_msgs::Point p1 = plan.poses[i].pose.position;
        geometry_msgs::Point p2 = plan.poses[i + 1].pose.position;
        distance += std::sqrt(std::pow((p2.x - p1.x), 2) + std::pow((p2.y - p1.y), 2));
    }
    return distance;
}

/*-----------------------------
overview: Return travel planning results using move_base's ROS service
argument: Start point of movement, end point of movement, travel planning
return: travel planning, returns true if route planning succeeds
-----------------------------*/
bool ViewpointEvaluatorServer::makePathPlan(const geometry_msgs::Pose &start, const geometry_msgs::Pose &goal, nav_msgs::Path &plan) {
    geometry_msgs::PoseStamped start_stamped;
    geometry_msgs::PoseStamped goal_stamped;
    start_stamped.header.frame_id = "map";
    goal_stamped.header.frame_id = "map";
    start_stamped.pose = start;
    goal_stamped.pose = goal;
    start_stamped.pose.position.z = 0.0;
    goal_stamped.pose.position.z = 0.0;

    nav_msgs::GetPlan make_plan_srv;
    make_plan_srv.request.start = start_stamped;
    make_plan_srv.request.goal = goal_stamped;
    make_plan_srv.request.tolerance = path_planning_tolerance;

    if(make_path_cli_.call(make_plan_srv)) {
        plan = make_plan_srv.response.plan;
    } else {
        std::cout << "Path planning failed" << std::endl;
        return false;
    }

    return true;
}

/*-----------------------------
overview: Calculate the distance traveled to each viewpoint candidate and set it in 'distances'
argument: None
return: None
set: distances
-----------------------------*/
void ViewpointEvaluatorServer::calcViewpointDistances(void) {
    geometry_msgs::Pose robot_pose = current_robot_pose_;
    // Make a path plan using a rosservie of move_base
    for(geometry_msgs::Pose candidate : candidates) {
        nav_msgs::Path plan;
        if(makePathPlan(robot_pose, candidate, plan)) {
            distances.push_back(calcTravelDistance(plan));
        } else {
            distances.push_back(-1.0);
        }
    }
}

/*-----------------------------
overview: Get viewpoint from 'generating_candidates' by service call
argument: None
return: Returns true if acquisition of viewpoint is successful
set: candidates
using: get_candidates_cli_(generating_candidates)
-----------------------------*/
bool ViewpointEvaluatorServer::getCandidates(void) {
    viewpoint_planner_3d::get_candidates srv;
    get_candidates_cli_.call(srv);
    if(srv.response.is_succeeded == true) {
        candidates = srv.response.candidates;
        ROS_INFO("Successful acquisition of viewpoint candidates");
        return true;
    } else {
        ROS_ERROR("Failed to call get_candidates");
        return false;
    }
}

/*-----------------------------
overview: Get next viewpoint(using ROS service)
argument: req, res (Take a look at get_next_viewpoint.srv)
return: is_succeeded - True if NBV (Next viewpoint) can be acquired normally
Ros searvice to use: get_viewpoint_candidates
-----------------------------*/
bool ViewpointEvaluatorServer::getNBV(viewpoint_planner_3d::get_next_viewpoint::Request &req,
                                      viewpoint_planner_3d::get_next_viewpoint::Response &res) {
    if(!getCandidates()) {
        res.is_succeeded = false;
        return false;
    }
    visualizationCandidates();
    calcViewpointDistances();
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