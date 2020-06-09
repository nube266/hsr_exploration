#include "viewpoint_evaluator/viewpoint_evaluator.h"

namespace viewpoint_evaluator_server {

/*-----------------------------
overview: Initialize a set of parameters and ROS services and subscribers
argument: ROS node handle
return: None
-----------------------------*/
ViewpointEvaluatorServer::ViewpointEvaluatorServer(ros::NodeHandlePtr node_handle) {
    nh_ = node_handle;
    // setParam();
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

} // namespace viewpoint_evaluator_server