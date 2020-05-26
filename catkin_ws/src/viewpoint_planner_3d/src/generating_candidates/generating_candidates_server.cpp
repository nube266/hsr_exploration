#include "generating_candidates/generating_candidates.h"

namespace generating_candidates_server {

GeneratingCandidatesServer::GeneratingCandidatesServer(ros::NodeHandlePtr node_handle) {
    nh_ = node_handle;
    setParam();
    map_sub_ = nh_->subscribe("/map", 1, &GeneratingCandidatesServer::mapCallback, this);
    gen_srv_ = nh_->advertiseService("/viewpoint_planner_3d/generating_candidates", &GeneratingCandidatesServer::generatingCandidates, this);
    ROS_INFO("Ready to generating_candidates_server");
}

GeneratingCandidatesServer::~GeneratingCandidatesServer() {
    cv::destroyAllWindows();
}

void GeneratingCandidatesServer::setParam() {
    ros::param::get("/generating_candidates/distance_between_candidates", distance_between_candidates);
}

bool GeneratingCandidatesServer::generatingCandidates(viewpoint_planner_3d::generating_candidates::Request &req,
                                                      viewpoint_planner_3d::generating_candidates::Response &res) {
    // Initialize viewpoint candidates
    candidates.clear();

    cv::namedWindow("test", cv::WINDOW_NORMAL);
    cv::imshow("test", map2img());
    cv::waitKey(0);

    res.is_succeeded = true;
    return true;
}

void GeneratingCandidatesServer::mapCallback(const nav_msgs::OccupancyGridConstPtr &map) {
    map_ = map;
}

cv::Mat GeneratingCandidatesServer::map2img(void) {
    geometry_msgs::Quaternion orientation = map_->info.origin.orientation;
    double yaw, pitch, roll;
    tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    mat.getEulerYPR(yaw, pitch, roll);
    double map_theta = yaw;
    int intensity;
    cv::Mat map_img = cv::Mat::zeros(cv::Size(map_->info.width, map_->info.height), CV_8UC1);

    for(unsigned int y = 0; y < map_->info.height; y++) {
        for(unsigned int x = 0; x < map_->info.width; x++) {
            unsigned int i = x + (map_->info.height - y - 1) * map_->info.width;
            intensity = 205;
            if(map_->data[i] >= 0 && map_->data[i] <= 100)
                intensity = round((float)(100.0 - map_->data[i]) * 2.55);
            map_img.at<unsigned char>(y, x) = intensity;
        }
    }
    return map_img.clone();
}

/*
costmap_2d::Costmap2D GeneratingCandidatesServer::obtainCostmap() {
    costmap_2d::Costmap2D::mutex_t *mutex = costmap_->getCostmap()->getMutex();
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*mutex);
    return *costmap_->getCostmap();
}
*/

} // namespace generating_candidates_server