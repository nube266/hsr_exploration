//
// Active Intelligent Systems Laboratory
// Toyohashi University of Technology
//
// Yusuke Miake
//

#include "./../include/avoidance_server.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "avoidance_server");
    ros::NodeHandlePtr nh = ros::NodeHandlePtr(new ros::NodeHandle);
    avoidance_server::AvoidanceServer avoidance_server(nh);
    ros::spin();
    return 0;
}
