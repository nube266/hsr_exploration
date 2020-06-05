#ifndef GENERATING_CANDIDATES_SERVER_H_
#define GENERATING_CANDIDATES_SERVER_H_

#include "viewpoint_planner_3d/generating_candidates.h"

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetMap.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/Image.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace generating_candidates_server {
class GeneratingCandidatesServer {
  private:
    /* Initial setting as ROS node */
    ros::NodeHandlePtr nh_;      // ROS node handle
    ros::ServiceServer gen_srv_; // ROS service that generates viewpoint candidates
    ros::Subscriber map_sub_;    // Subscriber updating map

    /* Variables for occupied grid map */
    std::vector<geometry_msgs::Pose> candidates; // Viewpoint candidate
    nav_msgs::OccupancyGridConstPtr map_;        // Occupancy grid map
    std::vector<cv::Point> frontier_centroids;   // Centroid of gravity of the frontier
    double distance_between_candidates = 0.3;    // Distance between center of gravity of frontier and viewpoint candidates[m]
    double candidate_yaw_resolution = 0.6;       // Candidate orientation resolution
    double distance_obstacle_candidate = 0.50;   // Minimum distance between obstacle and viewpoint candidate
    double min_frontier_length = 0.6;            // Minimum frontier size[m]
    double timeout = 10.0;                       // Timeout time when stopped by some processing[s]

    /*-----------------------------
    overview: Set of ROS parameters
    argument: None
    Return: None
    -----------------------------*/
    void setParam(void);

    /*-----------------------------
    overview: Generation of viewpoint candidates(using ROS service)
    argument: req, res (Take a look at generating_candidates.srv)
    return: True if the viewpoint candidate is successfully generated
    set: candidates
    -----------------------------*/
    bool generatingCandidates(viewpoint_planner_3d::generating_candidates::Request &req,
                              viewpoint_planner_3d::generating_candidates::Response &res);

    /*-----------------------------
    overview: Update map (map_) (using ROS subscribe)
    argument: map(Occupancy grid map)
    return: None
    set: map_(Occupancy grid map)
    -----------------------------*/
    void mapUpdate(const nav_msgs::OccupancyGridConstPtr &map);

    /*-----------------------------
    overview: Convert map from occupied grid map format to image format
    argument: None
    return: map(Image format)
    using: map_(Occupancy grid map format)
    -----------------------------*/
    cv::Mat map2img(void);

    /*-----------------------------
    overview: Detect and cluster frontiers
    argument: None
    return: None
    Set: frontier_centroids(vector<cv::Point>)
    Note: The centroid of the frontier is derived using labellingFrontier(function)
    -----------------------------*/
    void generateFrontierCluster(void);

    /*-----------------------------
    overview: Label frontier and calculate center of gravity and size
    argument: frontier_img
    return: None
    set: frontier_centroids(vector<cv::Point>)
    -----------------------------*/
    void labellingFrontier(cv::Mat frontier_img);

    /*-----------------------------
    overview: Convert real-world length (meter) to map length (pix)
    argument: length(meter)
    return: length(pix)
    -----------------------------*/
    int meter2pix(double length);

    /*-----------------------------
    overview: Convert map length (pix) to real-world length (meter)
    argument: length(pix)
    return: length(meter)
    -----------------------------*/
    double pix2meter(int length);

    /*-----------------------------
    overview: Wait until get the map successfully
    argument: None
    return: True if the map is successfully obtained
    -----------------------------*/
    bool waitMap(void);

    /*-----------------------------
    overview: Convert Euler to quaternion
    argument: Euler
    return: quaternion
    -----------------------------*/
    geometry_msgs::Quaternion rpy_to_geometry_quat(double roll, double pitch, double yaw);

    /*-----------------------------
    overview: Generate viewpoint candidates in a grid pattern
    argument: None
    return: True if the viewpoint candidate is successfully generated
    -----------------------------*/
    bool generateCandidateGridPattern(void);

  public:
    /*-----------------------------
    overview: Initialize a set of parameters and ROS services and subscribers
    argument: ROS node handle
    return: None
    -----------------------------*/
    GeneratingCandidatesServer(ros::NodeHandlePtr node_handle);

    /*-----------------------------
    overview: Processing at the end
    argument: None
    return: None
    -----------------------------*/
    ~GeneratingCandidatesServer();
};
} // namespace generating_candidates_server

#endif // GENERATING_CANDIDATES_SERVER_H_