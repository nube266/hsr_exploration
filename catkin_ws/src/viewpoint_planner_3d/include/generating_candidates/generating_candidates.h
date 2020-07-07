#ifndef GENERATING_CANDIDATES_SERVER_H_
#define GENERATING_CANDIDATES_SERVER_H_

// ros searvice
#include "viewpoint_planner_3d/generating_candidates.h"
#include "viewpoint_planner_3d/get_candidates.h"
#include "viewpoint_planner_3d/get_shortest_path_length.h"

// ros
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// opencv
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

// std
#include <chrono>
#include <cmath>
#include <float.h>
#include <iostream>
#include <memory>
#include <mutex>
#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <vector>

namespace generating_candidates_server {
class GeneratingCandidatesServer {
  private:
    /* Initial setting as ROS node */
    ros::NodeHandlePtr nh_;                           // ROS node handle
    ros::ServiceServer gen_srv_;                      // ROS service that generates viewpoint candidates
    ros::ServiceServer get_srv_;                      // ROS service that getter(viewpoint candidates)
    ros::Subscriber map_sub_;                         // Subscriber updating map
    ros::Subscriber odom_sub_;                        // ROS subscriber to get the current robot position
    ros::Subscriber global_costmap_sub_;              // Subscriber updating global costmap
    ros::Publisher candidates_marker_pub_;            // ROS publisher that candidates marker
    ros::ServiceClient clear_costmaps_cli_;           // Delete move_base cost map
    ros::ServiceClient get_shortest_path_length_cli_; // Get the shortest path length to each free cell

    /* Variables for occupied grid map */
    std::vector<geometry_msgs::Pose> candidates;     // Viewpoint candidate
    std::vector<float> distances;                    // Distance to each viewpoint candidate
    nav_msgs::OccupancyGridConstPtr map_;            // Occupancy grid map
    nav_msgs::OccupancyGridConstPtr global_costmap_; // Occupancy grid map(move_base)
    geometry_msgs::Pose current_robot_pose_;         // Current robot pose
    std::vector<cv::Point> frontier_centroids;       // Centroid of gravity of the frontier
    std::string odom_topic = "/hsrb/odom";           // Odometry topic name
    double distance_between_candidates = 0.3;        // Distance between center of gravity of frontier and viewpoint candidates[m]
    double candidate_yaw_resolution = 0.6;           // Candidate orientation resolution
    double distance_obstacle_candidate = 0.1;        // Size to expand Costmap [m]
    double min_frontier_length = 0.6;                // Minimum frontier size[m]
    double robot_head_pos_min = 1.00;                // Minimum of robot head position([m])
    double robot_head_pos_max = 1.69;                // Maximum of robot head position([m])
    double robot_head_candidate_resolution = 0.2;    // Resolution of viewpoint candidates in the height direction
    double max_free_space_noize_size = 0.1;          // Size of free space removed as noise by the opening process[m]
    double timeout = 10.0;                           // Timeout time when stopped by some processing[s]

    /*-----------------------------
    overview: Set of ROS parameters
    argument: None
    Return: None
    -----------------------------*/
    void setParam(void);

    /*-----------------------------
    overview: Get current robot position
    argument: None
    return: None
    set: odom
    -----------------------------*/
    void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);

    /*-----------------------------
    overview: Generation of viewpoint candidates(using ROS service)
    argument: req, res (Take a look at generating_candidates.srv)
    return: True if the viewpoint candidate is successfully generated
    set: candidates
    -----------------------------*/
    bool generatingCandidates(viewpoint_planner_3d::generating_candidates::Request &req,
                              viewpoint_planner_3d::generating_candidates::Response &res);

    /*-----------------------------
    overview: Return viewpoint candidates(using ROS service)
    argument: req, res (Take a look at get_candidates.srv)
    return: True if the viewpoint candidate is successfully got
    -----------------------------*/
    bool getCandidates(viewpoint_planner_3d::get_candidates::Request &req,
                       viewpoint_planner_3d::get_candidates::Response &res);

    /*-----------------------------
    overview: Update map (map_) (using ROS subscribe)
    argument: map(Occupancy grid map)
    return: None
    set: map_(Occupancy grid map)
    -----------------------------*/
    void mapUpdate(const nav_msgs::OccupancyGridConstPtr &map);

    /*-----------------------------
    overview: Update global_costmap (global_costmap_) (using ROS subscribe)
    argument: global_costmap(Occupancy grid map)
    return: None
    set: global_costmap_(Occupancy grid map)
    -----------------------------*/
    void globalCostmapUpdate(const nav_msgs::OccupancyGridConstPtr &global_costmap);

    /*-----------------------------
    overview: Convert map from occupied grid map format to image format
    argument: map(Occupancy grid map)
    return: map(Image format)
    -----------------------------*/
    cv::Mat map2img(nav_msgs::OccupancyGridConstPtr map);

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
    overview: Convert coordinates on the map to point on image
    argument: Coordinates on the image
    return: Coordinates on the map
    -----------------------------*/
    geometry_msgs::Point img_point2map_pose(int x, int y, double z);

    /*-----------------------------
    overview: Converts the coordinates on the map to the coordinates on the image
    argument: Coordinates on the map
    return: Coordinates on the image
    -----------------------------*/
    void map_pose2img_point(geometry_msgs::Pose, int &x, int &y);

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

    /*-----------------------------
    overview: Visualization of viewpoint candidates
    argument: None
    return: None
    -----------------------------*/
    void visualizationCandidates(void);

    /*-----------------------------
    overview: Calculate the distance to each viewpoint
    argument: map(cv::Mat)
    return: None
    set: distances
    -----------------------------*/
    void CalculateDistanceViewpoint(cv::Mat map_img);

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