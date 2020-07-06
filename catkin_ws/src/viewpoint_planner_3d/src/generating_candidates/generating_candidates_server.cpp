#include "generating_candidates/generating_candidates.h"

namespace generating_candidates_server {

/*-----------------------------
overview: Initialize a set of parameters and ROS services and subscribers
argument: ROS node handle
return: None
-----------------------------*/
GeneratingCandidatesServer::GeneratingCandidatesServer(ros::NodeHandlePtr node_handle) {
    nh_ = node_handle;
    setParam();
    map_sub_ = nh_->subscribe("/map", 1, &GeneratingCandidatesServer::mapUpdate, this);
    global_costmap_sub_ = nh_->subscribe("/move_base/global_costmap/costmap", 1, &GeneratingCandidatesServer::globalCostmapUpdate, this);
    gen_srv_ = nh_->advertiseService("/viewpoint_planner_3d/generating_candidates", &GeneratingCandidatesServer::generatingCandidates, this);
    get_srv_ = nh_->advertiseService("/viewpoint_planner_3d/get_candidates", &GeneratingCandidatesServer::getCandidates, this);
    clear_costmaps_cli_ = nh_->serviceClient<std_srvs::Empty>("/move_base/clear_costmaps", true);
    candidates_marker_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("/viewpoint_planner_3d/candidates_marker", 1);
    ROS_INFO("Ready to generating_candidates_server");
}

/*-----------------------------
overview: Processing at the end
argument: None
return: None
-----------------------------*/
GeneratingCandidatesServer::~GeneratingCandidatesServer() {
    cv::destroyAllWindows();
}

/*-----------------------------
overview: Set of ROS parameters
argument: None
Return: None
-----------------------------*/
void GeneratingCandidatesServer::setParam() {
    // Distance between candidate viewpoints [m]
    ros::param::get("/generating_candidates/distance_between_candidates", distance_between_candidates);
    ros::param::get("/generating_candidates/candidate_yaw_resolution", candidate_yaw_resolution);
    ros::param::get("/generating_candidates/distance_obstacle_candidate", distance_obstacle_candidate);
    ros::param::get("/generating_candidates/min_frontier_length", min_frontier_length);
    ros::param::get("/generating_candidates/robot_head_pos_min", robot_head_pos_min);
    ros::param::get("/generating_candidates/robot_head_pos_max", robot_head_pos_max);
    ros::param::get("/generating_candidates/robot_head_candidate_resolution", robot_head_candidate_resolution);
    ros::param::get("/generating_candidates/max_free_space_noize_size", max_free_space_noize_size);
    ros::param::get("/generating_candidates/timeout", timeout);
}

/*-----------------------------
overview: Generation of viewpoint candidates(using ROS service)
argument: req, res (Take a look at generating_candidates.srv)
return: True if the viewpoint candidate is successfully generated
set: candidates
-----------------------------*/
bool GeneratingCandidatesServer::generatingCandidates(viewpoint_planner_3d::generating_candidates::Request &req,
                                                      viewpoint_planner_3d::generating_candidates::Response &res) {
    // Wait to get the map
    if(waitMap() == false) {
        res.is_succeeded = false;
        return false;
    }

    // Generation of viewpoint candidates
    generateCandidateGridPattern();

    res.is_succeeded = true;
    return true;
}

/*-----------------------------
overview: Return viewpoint candidates(using ROS service)
argument: req, res (Take a look at get_candidates.srv)
using: candidates
-----------------------------*/
bool GeneratingCandidatesServer::getCandidates(viewpoint_planner_3d::get_candidates::Request &req,
                                               viewpoint_planner_3d::get_candidates::Response &res) {
    // Returns false if there is no viewpoint candidate
    if(candidates.empty() == true) {
        res, candidates = candidates;
        res.is_succeeded = false;
    } else {
        res.candidates = candidates;
        res.is_succeeded = true;
    }
    return true;
}

/*-----------------------------
overview: Update map (map_) (using ROS subscribe)
argument: map(Occupancy grid map)
return: None
set: map_(Occupancy grid map)
-----------------------------*/
void GeneratingCandidatesServer::mapUpdate(const nav_msgs::OccupancyGridConstPtr &map) {
    map_ = map;
}

/*-----------------------------
overview: Update global_costmap (global_costmap_) (using ROS subscribe)
argument: global_costmap(Occupancy grid map)
return: None
set: global_costmap_(Occupancy grid map)
-----------------------------*/
void GeneratingCandidatesServer::globalCostmapUpdate(const nav_msgs::OccupancyGridConstPtr &global_costmap) {
    global_costmap_ = global_costmap;
}

/*-----------------------------
overview: Convert map from occupied grid map format to image format
argument: map(Occupancy grid map)
return: map(Image format)
-----------------------------*/
cv::Mat GeneratingCandidatesServer::map2img(nav_msgs::OccupancyGridConstPtr map) {
    geometry_msgs::Quaternion orientation = map->info.origin.orientation;
    double yaw, pitch, roll;
    tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    mat.getEulerYPR(yaw, pitch, roll);
    double map_theta = yaw;
    int intensity;
    cv::Mat map_img = cv::Mat::zeros(cv::Size(map->info.width, map->info.height), CV_8UC1);
    for(unsigned int y = 0; y < map->info.height; y++) {
        for(unsigned int x = 0; x < map->info.width; x++) {
            unsigned int i = x + (map->info.height - y - 1) * map->info.width;
            intensity = 205;
            if(map->data[i] >= 0 && map->data[i] <= 100)
                intensity = round((float)(100.0 - map->data[i]) * 2.55);
            map_img.at<unsigned char>(y, x) = intensity;
        }
    }
    return map_img.clone();
}

/*-----------------------------
overview: Detect and cluster frontiers
argument: None
return: None
Set: frontier_centroids(vector<cv::Point>)
Note: The centroid of the frontier is derived using labellingFrontier(function)
-----------------------------*/
void GeneratingCandidatesServer::generateFrontierCluster(void) {
    cv::Mat map_img = map2img(map_);
    cv::Mat frontier_img, mask_unknown_img, mask_free_img;
    cv::inRange(map_img, cv::Scalar(1, 1, 1), cv::Scalar(254, 254, 254), mask_unknown_img);
    cv::inRange(map_img, cv::Scalar(250, 250, 250), cv::Scalar(256, 256, 256), mask_free_img);
    cv::erode(mask_free_img, mask_free_img, cv::Mat(), cv::Point(-1, 1), 3);
    cv::dilate(mask_unknown_img, mask_unknown_img, cv::Mat(), cv::Point(-1, 1), 4);
    cv::bitwise_and(mask_unknown_img, mask_free_img, frontier_img);
    labellingFrontier(frontier_img);
    cv::namedWindow("map", cv::WINDOW_NORMAL);
    cv::namedWindow("frontier", cv::WINDOW_NORMAL);
    cv::imshow("map", map2img(map_));
    cv::imshow("frontier", frontier_img);
    cv::waitKey(0);
}

/*-----------------------------
overview: Label frontier and calculate center of gravity and size
argument: frontier_img
return: None
set: frontier_centroids(vector<cv::Point>)
-----------------------------*/
void GeneratingCandidatesServer::labellingFrontier(cv::Mat frontier_img) {
    cv::Mat label_img, stats, centroids;
    int nLabels = cv::connectedComponentsWithStats(frontier_img, label_img, stats, centroids, 8, 4);
    int min_area_size = meter2pix(min_frontier_length);
    for(int i = 0; i < nLabels - 1; ++i) {
        double *param_cen = centroids.ptr<double>(i + 1);
        int *param_sta = stats.ptr<int>(i + 1);
        int x = static_cast<int>(param_cen[0]);
        int y = static_cast<int>(param_cen[1]);
        int area_size = param_sta[cv::ConnectedComponentsTypes::CC_STAT_AREA];
        if(area_size >= min_area_size) {
            frontier_centroids.push_back(cv::Point(x, y));
        }
    }

    std::vector<cv::Vec3b> colors(nLabels);
    colors[0] = cv::Vec3b(0, 0, 0);
    for(int label = 1; label < nLabels; label++)
        colors[label] = cv::Vec3b((rand() & 255), (rand() & 255), (rand() & 255));
    cv::Mat dst_img(frontier_img.size(), CV_8UC3);
    for(int i = 0; i < dst_img.rows; ++i) {
        int *lb = label_img.ptr<int>(i);
        cv::Vec3b *pix = dst_img.ptr<cv::Vec3b>(i);
        for(int j = 0; j < dst_img.cols; ++j) {
            pix[j] = colors[lb[j]];
        }
    }
    for(int i = 0; i < nLabels - 1; ++i) {
        cv::circle(dst_img, frontier_centroids[i], 3, cv::Scalar(0, 0, 255), -1);
    }
    cv::namedWindow("dst_img", cv::WINDOW_NORMAL);
    cv::imshow("dst_img", dst_img);
}

/*-----------------------------
overview: Convert real-world length (meter) to map length (pix)
argument: length(meter)
return: length(pix)
-----------------------------*/
int GeneratingCandidatesServer::meter2pix(double length) {
    return (int)(std::round(length / map_->info.resolution));
}

/*-----------------------------
overview: Convert coordinates on the image to point on map
argument: Coordinates on the image
return: point
-----------------------------*/
geometry_msgs::Point GeneratingCandidatesServer::img_point2map_pose(int x, int y, double z) {
    geometry_msgs::Point position;
    geometry_msgs::Pose origin = map_->info.origin;
    position.x = (double)x * map_->info.resolution + origin.position.x;
    position.y = (double)(map_->info.height - y) * map_->info.resolution + origin.position.y;
    position.z = z;
    return position;
}

/*-----------------------------
overview: Wait until get the map successfully
argument: None
return: True if the map is successfully obtained
-----------------------------*/
bool GeneratingCandidatesServer::waitMap(void) {
    std::chrono::system_clock::time_point start, current;
    start = std::chrono::system_clock::now();
    while(1) {
        ros::spinOnce();
        if(map_ != NULL)
            return true;
        current = std::chrono::system_clock::now();
        double elapsed_time = static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(current - start).count() / 1000000.0);
        if(elapsed_time >= timeout) {
            ROS_INFO("/map is not subscribed yet");
            return false;
        }
    }
}

/*-----------------------------
overview: Convert Euler to quaternion
argument: Euler
return: quaternion
-----------------------------*/
geometry_msgs::Quaternion GeneratingCandidatesServer::rpy_to_geometry_quat(double roll, double pitch, double yaw) {
    tf::Quaternion quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    return geometry_quat;
}

/*-----------------------------
overview: Generate viewpoint candidates in a grid pattern
argument: None
return: True if the viewpoint candidate is successfully generated
-----------------------------*/
bool GeneratingCandidatesServer::generateCandidateGridPattern(void) {
    // Initialize viewpoint candidates
    candidates.clear();
    cv::Mat map_img = map2img(map_);
    // Opening process(noise removal)
    cv::erode(map_img, map_img, cv::Mat(), cv::Point(-1, 1), meter2pix(max_free_space_noize_size));
    cv::dilate(map_img, map_img, cv::Mat(), cv::Point(-1, 1), meter2pix(max_free_space_noize_size));
    // Convert Costmap to image and contract
    cv::Mat occupancy_img;
    cv::threshold(map2img(global_costmap_), occupancy_img, 5, 255, cv::THRESH_BINARY);
    cv::erode(occupancy_img, occupancy_img, cv::Mat(), cv::Point(-1, 1), meter2pix(distance_obstacle_candidate));
    // Generation of viewpoint candidates
    int grid_size = meter2pix(distance_between_candidates);
    for(int y = 0; y < map_img.rows; y += grid_size) {
        for(int x = 0; x < map_img.cols; x += grid_size) {
            unsigned char intensity = map_img.at<unsigned char>(y, x);
            if(intensity == 255 && occupancy_img.at<unsigned char>(y, x) == 255) { // white(free)
                geometry_msgs::Pose pose;
                for(double yaw = -180; yaw < 180; yaw += candidate_yaw_resolution) {
                    for(double z = robot_head_pos_min; z < robot_head_pos_max; z += robot_head_candidate_resolution) {
                        pose.position = img_point2map_pose(x, y, z);
                        pose.orientation = rpy_to_geometry_quat(0.0, 0.0, yaw * (M_PI / 180));
                        candidates.push_back(pose);
                    }
                }
            }
        }
    }
    // Calculate the distance to each viewpoint
    CalculateDistanceViewpoint(map_img);
}

/*-----------------------------
overview: Visualization of viewpoint candidates
argument: None
return: None
-----------------------------*/
void GeneratingCandidatesServer::visualizationCandidates(void) {
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(candidates.size());
    int id = 0;
    for(geometry_msgs::Pose candidate : candidates) {
        marker_array.markers[id].header.frame_id = "/map";
        marker_array.markers[id].header.stamp = ros::Time::now();
        marker_array.markers[id].ns = "/candidate";
        marker_array.markers[id].id = id;
        marker_array.markers[id].type = visualization_msgs::Marker::ARROW;
        marker_array.markers[id].action = visualization_msgs::Marker::ADD;
        marker_array.markers[id].lifetime = ros::Duration(0);
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

/*-----------------------------
overview: Calculate the distance to each viewpoint
argument: map(cv::Mat)
return: None
set: distances
-----------------------------*/
void GeneratingCandidatesServer::CalculateDistanceViewpoint(cv::Mat map_img) {
    // Generate adjacency list from free space
    // example:
    // graph
    // 1----2---4
    //   |--3
    // data
    // node1: 1 1 2 2 3 4
    // node2: 2 3 1 4 1 2
    std::vector<int> node1;
    std::vector<int> node2;
    for(int y = 0; y < map_img.rows; ++y) {
        for(int x = 0; x < map_img.cols; ++x) {
            if(map_img.at<unsigned char>(y, x) == 0)
                continue;
            if(x != 0) {
                if(map_img.at<unsigned char>(y, x - 1) == 255) {
                    node1.push_back(x + map_img.cols * y);
                    node2.push_back(x - 1 + map_img.cols * y);
                }
            }
            if(x != map_img.cols - 1) {
                if(map_img.at<unsigned char>(y, x + 1) == 255) {
                    node1.push_back(x + map_img.cols * y);
                    node2.push_back(x + 1 + map_img.cols * y);
                }
            }
            if(y != 0) {
                if(map_img.at<unsigned char>(y - 1, x) == 255) {
                    node1.push_back(x + map_img.cols * y);
                    node2.push_back(x + map_img.cols * (y - 1));
                }
            }
            if(y != map_img.rows - 1) {
                if(map_img.at<unsigned char>(y + 1, x) == 255) {
                    node1.push_back(x + map_img.cols * y);
                    node2.push_back(x + map_img.cols * (y + 1));
                }
            }
        }
    }
}

} // namespace generating_candidates_server