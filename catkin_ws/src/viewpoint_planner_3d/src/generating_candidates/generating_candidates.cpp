// Self-build class
#include "generating_candidates/generating_candidates.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "generating_candidates_server");
    ros::NodeHandlePtr nh_ = ros::NodeHandlePtr(new ros::NodeHandle);
    ros::Rate r(0.5);
    generating_candidates_server::GeneratingCandidatesServer server(nh_);
    ros::spin();
    return 0;
}