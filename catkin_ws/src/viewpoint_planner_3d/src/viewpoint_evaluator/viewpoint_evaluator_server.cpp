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
}

/*-----------------------------
overview: Get next viewpoint(using ROS service)
argument: req, res (Take a look at get_next_viewpoint.srv)
return: is_succeeded - True if NBV (Next viewpoint) can be acquired normally
Ros searvice to use: get_viewpoint_candidates
-----------------------------*/
bool ViewpointEvaluatorServer::getNBV(viewpoint_planner_3d::get_next_viewpoint::Request &req,
                                      viewpoint_planner_3d::get_next_viewpoint::Response &res) {
    return true;
}

} // namespace viewpoint_evaluator_server