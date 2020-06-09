// Self-build class
#include "viewpoint_evaluator/viewpoint_evaluator.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "viewpoint_evaluator_server");
    ros::NodeHandlePtr nh_ = ros::NodeHandlePtr(new ros::NodeHandle);
    ros::Rate r(20);
    viewpoint_evaluator_server::ViewpointEvaluatorServer server(nh_);
    ros::spin();
    return 0;
}