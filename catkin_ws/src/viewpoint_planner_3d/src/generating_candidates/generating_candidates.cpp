// Self-build class
#include "generating_candidates/generating_candidates.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "generating_candidates_server");
    ros::NodeHandlePtr nh_ = ros::NodeHandlePtr(new ros::NodeHandle);
    double map_update_frequency = 1.0;
    ros::param::get("/generating_candidates/map_update_frequency", map_update_frequency);
    ros::Rate r(map_update_frequency);
    generating_candidates_server::GeneratingCandidatesServer server(nh_);
    ros::spin();
    return 0;
}