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
    raycast_marker_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("/viewpoint_planner_3d/raycast_marker", 1);
    clear_costmaps_cli_ = nh_->serviceClient<std_srvs::Empty>("/move_base/clear_costmaps", true);
    // Initialize Subscriber and Publisher
    odom_sub_ = nh_->subscribe(odom_topic, 1, &ViewpointEvaluatorServer::odomCallback, this);
    octomap_sub_ = nh_->subscribe("/octomap_binary", 1, &ViewpointEvaluatorServer::subscribeOctomap, this);
    // Initialize get tree time
    current_get_octree_time = std::chrono::system_clock::now();
    previous_get_octree_time = current_get_octree_time;
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
return: None
-----------------------------*/
void ViewpointEvaluatorServer::setParam() {
    ros::param::get("/viewpoint_evaluator/timeout", timeout);
    ros::param::get("/viewpoint_evaluator/candidate_marker_lifetime", candidate_marker_lifetime);
    ros::param::get("/viewpoint_evaluator/odom_topic", odom_topic);
    ros::param::get("/viewpoint_evaluator/sensor_max_range", sensor_max_range);
    ros::param::get("/viewpoint_evaluator/sensor_horizotal_range", sensor_horizontal_range);
    ros::param::get("/viewpoint_evaluator/sensor_vertical_range", sensor_vertical_range);
    ros::param::get("/viewpoint_evaluator/lamda", lamda_);
    ros::param::get("/viewpoint_evaluator/raycast_horizotal_resolution", raycast_horizontal_resolution_);
    ros::param::get("/viewpoint_evaluator/raycast_vertical_resolution", raycast_vertical_resolution_);
}

/*-----------------------------
overview: Get current robot position
argument: None
return: None
set: odom
-----------------------------*/
void ViewpointEvaluatorServer::odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
    nav_msgs::Odometry odom = *odom_msg;
    current_robot_pose_ = odom.pose.pose;
}

/*-----------------------------
overview: Get the octomap
argument: None
return: None
set: octree_(Octree)
-----------------------------*/
void ViewpointEvaluatorServer::subscribeOctomap(const octomap_msgs::Octomap &msg) {
    octree_mutex.lock();
    // Convert the binary message of OctoMap into an octomap::Octee
    if(octree_ != nullptr) {
        delete octree_;
    }
    octomap::AbstractOcTree *tmp = octomap_msgs::binaryMsgToMap(msg);
    octree_ = dynamic_cast<octomap::OcTree *>(tmp);
    current_get_octree_time = std::chrono::system_clock::now();
    octree_mutex.unlock();
}

/*-----------------------------
overview: Wait until Octomap can be acquired
argument: None
return: Returns false if Octomap cannot be obtained
set: octree_(Octree)
-----------------------------*/
bool ViewpointEvaluatorServer::waitGetOctomap(void) {
    // Wait for Octomap to be updated
    std::chrono::system_clock::time_point start, current;
    start = std::chrono::system_clock::now();
    float elapsed_time = 0.0;
    while(elapsed_time < timeout) {
        ros::spinOnce();
        if(current_get_octree_time != previous_get_octree_time) {
            previous_get_octree_time = current_get_octree_time;
            break;
        }
        current = std::chrono::system_clock::now();
        elapsed_time = static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(current - start).count() / 1000000.0);
    }
    if(elapsed_time >= timeout) {
        std::cout << "[Warning] map is not updated" << std::endl;
    }
    if(octree_ == nullptr) {
        std::cout << "Failed to get to octree" << std::endl;
        return false;
    }
    return true;
}

/*-----------------------------
overview: Get viewpoint from 'generating_candidates' by service call
argument: None
return: Returns true if acquisition of viewpoint is successful
set: candidates, distances
using: get_candidates_cli_(generating_candidates)
-----------------------------*/
bool ViewpointEvaluatorServer::getCandidates(void) {
    viewpoint_planner_3d::get_candidates srv;
    get_candidates_cli_.call(srv);
    if(srv.response.is_succeeded == true) {
        candidates = srv.response.candidates;
        distances = srv.response.distances;
        ROS_INFO("Successful acquisition of viewpoint candidates");
        return true;
    } else {
        ROS_ERROR("Failed to call get_candidates");
        return false;
    }
}

/*-----------------------------
overview: Convert euler angle to quaternion
argument: Euler(roll, pitch, yaw)
return: quaternion
-----------------------------*/
geometry_msgs::Quaternion ViewpointEvaluatorServer::rpy_to_geometry_quat(double roll, double pitch, double yaw) {
    tf::Quaternion quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    return geometry_quat;
}

/*-----------------------------
overview: Convert quaternion to euler angle
argument: quaternion
return: Euler(roll, pitch, yaw)
-----------------------------*/
void ViewpointEvaluatorServer::geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat) {
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

/*-----------------------------
overview: Convert degree to radian
argument: degree
return: radian
-----------------------------*/
double ViewpointEvaluatorServer::deg2rad(double deg) {
    return deg * (M_PI / 180);
}

/*-----------------------------
overview: Raycast endpoint visualization
argument: viewpoint and the end point of the raycast at this point
return: None
-----------------------------*/
void ViewpointEvaluatorServer::visualizationRaycastEndpoint(geometry_msgs::Pose viewpoint, std::vector<geometry_msgs::Point> end_points) {
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(end_points.size() + 1);
    int id = 0;
    for(geometry_msgs::Point end_point : end_points) {
        visualization_msgs::Marker marker;
        marker_array.markers[id].header.frame_id = "/map";
        marker_array.markers[id].header.stamp = ros::Time::now();
        marker_array.markers[id].ns = "/end_points";
        marker_array.markers[id].id = id;
        marker_array.markers[id].type = visualization_msgs::Marker::CUBE;
        marker_array.markers[id].action = visualization_msgs::Marker::ADD;
        marker_array.markers[id].lifetime = ros::Duration(5.0);
        marker_array.markers[id].pose = viewpoint;
        marker_array.markers[id].pose.position.x = end_point.x;
        marker_array.markers[id].pose.position.y = end_point.y;
        marker_array.markers[id].pose.position.z = end_point.z;
        marker_array.markers[id].scale.x = 0.05;
        marker_array.markers[id].scale.y = 0.05;
        marker_array.markers[id].scale.z = 0.05;
        marker_array.markers[id].color.r = 0.0f;
        marker_array.markers[id].color.g = 1.0f;
        marker_array.markers[id].color.b = 0.0f;
        marker_array.markers[id].color.a = 1.0f;
        id++;
    }
    visualization_msgs::Marker marker;
    marker_array.markers[id].header.frame_id = "/map";
    marker_array.markers[id].header.stamp = ros::Time::now();
    marker_array.markers[id].ns = "/viewpoint";
    marker_array.markers[id].id = id;
    marker_array.markers[id].type = visualization_msgs::Marker::ARROW;
    marker_array.markers[id].action = visualization_msgs::Marker::ADD;
    marker_array.markers[id].lifetime = ros::Duration(5.0);
    marker_array.markers[id].pose = viewpoint;
    marker_array.markers[id].scale.x = 0.5;
    marker_array.markers[id].scale.y = 0.05;
    marker_array.markers[id].scale.z = 0.05;
    marker_array.markers[id].color.r = 1.0f;
    marker_array.markers[id].color.g = 0.0f;
    marker_array.markers[id].color.b = 0.0f;
    marker_array.markers[id].color.a = 1.0f;
    raycast_marker_pub_.publish(marker_array);
}

/*-----------------------------
overview: Returns the end points of the raycast
argument: viewpoint(pose),
return: End points of the raycast
-----------------------------*/
std::vector<geometry_msgs::Point> ViewpointEvaluatorServer::computeRayDirections(geometry_msgs::Pose viewpoint) {
    // Convert viewpoint direction to Euler angle
    double roll, pitch, yaw;
    geometry_quat_to_rpy(roll, pitch, yaw, viewpoint.orientation);
    // Set sensor parameter and resolution
    double max_range = sensor_max_range;
    double horizotal_range = deg2rad(sensor_horizontal_range);
    double vertical_range = deg2rad(sensor_vertical_range);
    double octomap_resolution;
    ros::param::get("/octomap_server/resolution", octomap_resolution);
    double horizotal_resolution = deg2rad(raycast_horizontal_resolution_);
    double vertical_resolution = deg2rad(raycast_vertical_resolution_);
    // Upper and lower limits of vertical angle
    double theta_min = M_PI / 2 - vertical_range / 2 + pitch;
    double theta_max = M_PI / 2 + vertical_range / 2 + pitch;
    // Upper and lower limits of horizontal angle
    double fai_min = yaw - horizotal_range / 2;
    double fai_max = yaw + horizotal_range / 2;
    if(fai_max >= 2 * M_PI)
        fai_max = fai_max - 2 * M_PI;
    if(fai_max < 0.0)
        fai_max = fai_max + 2 * M_PI;
    if(fai_min >= 2 * M_PI)
        fai_min = fai_min - 2 * M_PI;
    if(fai_min < 0.0)
        fai_min = fai_min + 2 * M_PI;
    if(fai_max < fai_min) {
        double temp = fai_max;
        fai_max = fai_min;
        fai_min = temp;
    }
    // Calculate end points of ray_cast
    std::vector<geometry_msgs::Point> end_points;
    if(abs(fai_max - fai_min - horizotal_range) > abs(fai_min + 2 * M_PI - fai_max - horizotal_range)) {
        // When the sensor range passes the singular point (0[deg])
        double fai = fai_max;
        while(fai < fai_min && fai >= fai_max) {
            for(double theta = theta_min; theta < theta_max; theta += vertical_resolution) {
                geometry_msgs::Point end_point;
                end_point.x = viewpoint.position.x + max_range * sin(theta) * cos(fai);
                end_point.y = viewpoint.position.y + max_range * sin(theta) * sin(fai);
                end_point.z = viewpoint.position.z + max_range * cos(theta);
                end_points.push_back(end_point);
            }
            fai += horizotal_resolution;
        }
    } else {
        double fai = fai_min;
        while(fai >= fai_min && fai < fai_max) {
            for(double theta = theta_min; theta < theta_max; theta += vertical_resolution) {
                geometry_msgs::Point end_point;
                end_point.x = viewpoint.position.x + max_range * sin(theta) * cos(fai);
                end_point.y = viewpoint.position.y + max_range * sin(theta) * sin(fai);
                end_point.z = viewpoint.position.z + max_range * cos(theta);
                end_points.push_back(end_point);
            }
            fai += horizotal_resolution;
        }
    }
    // visualizationRaycastEndpoint(viewpoint, end_points);
    // cv::Mat image = cv::Mat::zeros(500, 500, CV_8UC3);
    // cv::imshow("image", image);
    // cv::waitKey(0);
    return end_points;
}

/*-----------------------------
overview: raycast visualization
argument: octomap node keys
return: None
-----------------------------*/
void ViewpointEvaluatorServer::raycastVisualization(octomap::KeySet keys) {
    visualization_msgs::MarkerArray marker_array;
    int id = 0;
    marker_array.markers.resize(keys.size());
    for(auto key = keys.begin(); key != keys.end(); ++key) {
        octomap::point3d point = octree_->keyToCoord(*key);
        marker_array.markers[id].header.frame_id = "/map";
        marker_array.markers[id].header.stamp = ros::Time::now();
        marker_array.markers[id].ns = "/raycast";
        marker_array.markers[id].id = id;
        marker_array.markers[id].type = visualization_msgs::Marker::CUBE;
        marker_array.markers[id].action = visualization_msgs::Marker::ADD;
        marker_array.markers[id].lifetime = ros::Duration(5.0);
        marker_array.markers[id].pose.position.x = point.x();
        marker_array.markers[id].pose.position.y = point.y();
        marker_array.markers[id].pose.position.z = point.z();
        double octomap_resolution;
        ros::param::get("/octomap_server/resolution", octomap_resolution);
        marker_array.markers[id].scale.x = octomap_resolution;
        marker_array.markers[id].scale.y = octomap_resolution;
        marker_array.markers[id].scale.z = octomap_resolution;
        marker_array.markers[id].color.r = 0.0f;
        marker_array.markers[id].color.g = 1.0f;
        marker_array.markers[id].color.b = 0.0f;
        marker_array.markers[id].color.a = 1.0f;
        raycast_marker_pub_.publish(marker_array);
        id++;
    }
    raycast_marker_pub_.publish(marker_array);
}

/*-----------------------------
overview: Calculate the region of ​​the unknown that can be observed from the viewpoint candidate
argument: viewpoint(pose)
return: Number of Unknown voxels
-----------------------------*/
int ViewpointEvaluatorServer::countUnknownObservable(geometry_msgs::Pose viewpoint) {
    octomap::KeySet unknown;
    // Get end points of raycast
    std::vector<geometry_msgs::Point> ray_end_points = computeRayDirections(viewpoint);
    // Preparing for speed up ray casting
    octomap::OcTreeKey justRay, previousRay;
    bool isFirst = true;
    // Preparing for visualization
    octomap::KeySet visualization_voxel_key;
    // Count the number of unknown voxels with raycast
    for(auto it = ray_end_points.begin(); it != ray_end_points.end(); ++it) {
        octomap::point3d end(it->x, it->y, it->z);
        // If the ray passes through the same cell as the previous time, skip the following process
        justRay = octree_->coordToKey(end);
        if(isFirst) {
            previousRay = justRay;
            isFirst = false;
        } else {
            if(justRay == previousRay)
                continue;
            else
                previousRay = justRay;
        }
        // Ray casting
        octomap::KeyRay ray;
        octomap::point3d origin(viewpoint.position.x, viewpoint.position.y, viewpoint.position.z);
        bool success = octree_->computeRayKeys(origin, end, ray);
        // Count the number of unknown cell observable
        if(success && ray.size() != 0) {
            for(auto _it = ray.begin(); _it != ray.end(); ++_it) {
                // Check whether ray hits an unknown cell or occupied cell
                octomap::OcTreeNode *node = nullptr;
                node = octree_->search(*_it);
                visualization_voxel_key.insert(*_it);
                if(node == nullptr) {
                    unknown.insert(*_it);
                } else if(octree_->isNodeOccupied(node)) {
                    break;
                }
            }
        }
    }
    // raycastVisualization(visualization_voxel_key);
    // cv::Mat image = cv::Mat::zeros(500, 500, CV_8UC3);
    // cv::imshow("image", image);
    // cv::waitKey(0);

    return unknown.size();
} // namespace viewpoint_evaluator_server

/*-----------------------------
overview: Evaluate viewpoint candidates
argument: None
return: Next viewpoint
using: candidates, distances
-----------------------------*/
geometry_msgs::Pose ViewpointEvaluatorServer::evaluateViewpoints(void) {
    // Initialize
    std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
    std::vector<double> gains;
    gains.resize(candidates.size());
#pragma omp parallel for schedule(guided, 1), default(shared)
    for(int i = 0; i < candidates.size(); ++i) {
        int unknwonNum = countUnknownObservable(candidates[i]);
        gains[i] = unknwonNum * std::exp(-1.0 * lamda_ * distances[i]);
    }
    double max_gain = 0;
    geometry_msgs::Pose next_viewpoint;
    for(int i = 0; i < candidates.size(); ++i) {
        if(gains[i] >= max_gain) {
            max_gain = gains[i];
            next_viewpoint = candidates[i];
        }
    }
    std::cout << "------------" << std::endl;
    std::chrono::system_clock::time_point current = std::chrono::system_clock::now();
    float elapsed_time = static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(current - start).count() / 1000000.0);
    std::cout << "candidate num: " << candidates.size() << std::endl;
    std::cout << "elapsed time: " << elapsed_time << std::endl;
    std::cout << "------------" << std::endl;
    return next_viewpoint;
}

/*-----------------------------
overview: Get next viewpoint(using ROS service)
argument: req, res (Take a look at get_next_viewpoint.srv)
return: is_succeeded - True if NBV (Next viewpoint) can be acquired normally
Ros searvice to use: get_viewpoint_candidates
-----------------------------*/
bool ViewpointEvaluatorServer::getNBV(viewpoint_planner_3d::get_next_viewpoint::Request &req,
                                      viewpoint_planner_3d::get_next_viewpoint::Response &res) {
    // Initialize
    std::vector<geometry_msgs::Pose>().swap(candidates);
    std::vector<float>().swap(distances);
    // Changed to decide whether to wait for Octomap update with ROS param
    if(!waitGetOctomap()) {
        res.is_succeeded = false;
        return false;
    }
    if(!getCandidates()) {
        res.is_succeeded = false;
        return false;
    }
    visualizationCandidates();
    geometry_msgs::Pose next_viewpoint = evaluateViewpoints();
    double viewpoint_height = next_viewpoint.position.z;
    next_viewpoint.position.z = 0.0;
    res.next_viewpoint = next_viewpoint;
    res.is_succeeded = true;
    return true;
} // namespace viewpoint_evaluator_server

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