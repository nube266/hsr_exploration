#ifndef VIEWPOINT_EVALUATOR_SERVER_H_
#define VIEWPOINT_EVALUATOR_SERVER_H_

// ros searvice
#include "viewpoint_planner_3d/get_candidates.h"
#include "viewpoint_planner_3d/get_next_viewpoint.h"

// ros
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <navfn/navfn_ros.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// OctoMap
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

// OpenMP
#ifdef _OPENMP
#include <omp.h>
#endif

// opencv
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

// std
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace viewpoint_evaluator_server {
class ViewpointEvaluatorServer {
  private:
    /* Initial setting as ROS node */
    ros::NodeHandlePtr nh_;                 // ROS node handle
    ros::ServiceServer get_nbv_srv_;        // ROS service that gets next viewpoint
    ros::ServiceClient get_candidates_cli_; // ROS service client that gets view viewpoint candidates
    ros::Publisher candidates_marker_pub_;  // ROS publisher that candidates marker
    ros::Publisher raycast_marker_pub_;     // ROS publisher for displaying raycast markers
    ros::Subscriber odom_sub_;              // ROS subscriber to get the current robot position
    ros::Subscriber octomap_sub_;           // ROS subscriber to get the Octomap(Octree)
    ros::ServiceClient clear_costmaps_cli_; // Delete move_base cost map

    /* Variables used to evaluate viewpoint candidates */
    geometry_msgs::Pose current_robot_pose_;     // Current robot pose
    std::vector<geometry_msgs::Pose> candidates; // Viewpoint candidates
    std::vector<float> distances;                // Distance to each viewpoint candidates

    /* Octomap */
    octomap::OcTree *octree_ = nullptr;
    std::mutex octree_mutex;
    std::chrono::system_clock::time_point current_get_octree_time;  // Time to get the latest Octree
    std::chrono::system_clock::time_point previous_get_octree_time; // Last time Octtree was acquired

    /* Parameter */
    double sensor_max_range = 3.5;               // Maximum sensor range (distance) [m]
    double sensor_horizontal_range = 58.0;       // Horizontal sensor viewing angle [deg]
    double sensor_vertical_range = 45.0;         // Vertical sensor viewing angle [deg]
    double timeout = 10.0;                       // Timeout time when stopped by some processing[s]
    double candidate_marker_lifetime = 5.0;      // Display time when the marker is visualized
    std::string odom_topic = "/hsrb/odom";       // Odometry topic name
    double raycast_horizontal_resolution_ = 5.0; // Horizontal resolution of raycast
    double raycast_vertical_resolution_ = 5.0;   // Vertical resolution of raycast
    double robot_movement_speed = 0.2;           // Robot movement speed[m] (HSRB default speed: 0.2[m])
    double offset_gain = 2.0;                    // Offset when calculating gain(Usually the time taken for perspective planning[sec])
    double lamda_ = 0.2;                         // This parameter is related to the distance to move when calculating the NBV. If this parameter is set to 0, the movement distance is ignored.

    /*-----------------------------
    overview: Set of ROS parameters
    argument: None
    return: None
    -----------------------------*/
    void setParam(void);

    /*-----------------------------
    overview: Get current robot position
    argument: None
    return: None
    set: odom_
    -----------------------------*/
    void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);

    /*-----------------------------
    overview: Get the octomap
    argument: None
    return: None
    set: octree_(Octree)
    -----------------------------*/
    void subscribeOctomap(const octomap_msgs::Octomap &msg);

    /*-----------------------------
    overview: Wait until Octomap can be acquired
    argument: None
    return: Returns false if Octomap cannot be obtained
    set: octree_(Octree)
    -----------------------------*/
    bool waitGetOctomap(void);

    /*-----------------------------
    overview: Get viewpoint from 'generating_candidates' by service call
    argument: None
    return: None
    set: candidates
    using: get_candidates_cli_(generating_candidates)
    -----------------------------*/
    bool getCandidates(void);

    /*-----------------------------
    overview: Convert euler angle to quaternion
    argument: Euler(roll, pitch, yaw)
    return: quaternion
    -----------------------------*/
    geometry_msgs::Quaternion rpy_to_geometry_quat(double roll, double pitch, double yaw);

    /*-----------------------------
    overview: Convert quaternion to euler angle
    argument: quaternion
    return: Euler(roll, pitch, yaw)
    -----------------------------*/
    void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat);

    /*-----------------------------
    overview: Convert degree to radian
    argument: degree
    return: radian
    -----------------------------*/
    double deg2rad(double deg);

    /*-----------------------------
    overview: Raycast endpoint visualization
    argument: viewpoint and the end point of the raycast at this point
    return: None
    -----------------------------*/
    void visualizationRaycastEndpoint(geometry_msgs::Pose viewpoint, std::vector<geometry_msgs::Point> end_points);

    /*-----------------------------
    overview: Returns the end points of the raycast
    argument: viewpoint(pose),
    return: End points of the raycast
    -----------------------------*/
    std::vector<geometry_msgs::Point> computeRayDirections(geometry_msgs::Pose viewpoint);

    /*-----------------------------
    overview: raycast visualization
    argument: octomap node keys
    return: None
    -----------------------------*/
    void raycastVisualization(octomap::KeySet keys);

    /*-----------------------------
    overview: Calculate the region of ​​the unknown that can be observed from the viewpoint candidate
    argument: viewpoint(pose)
    return: Number of Unknown voxels
    -----------------------------*/
    int countUnknownObservable(geometry_msgs::Pose viewpoint);

    /*-----------------------------
    overview: Calculate the maximum observable unknown voxels based on the sensor model
    argument: None
    return: Maximum observable volume of unknown voxels (number of voxels)
    -----------------------------*/
    int maxUnknownObservable(void);

    /*-----------------------------
    overview: Evaluate viewpoint candidates
    argument: None
    return: Next viewpoint
    using: candidates, distances
    -----------------------------*/
    geometry_msgs::Pose evaluateViewpoints(void);

  public:
    /*-----------------------------
    overview: Initialize a set of parameters and ROS services and subscribers
    argument: ROS node handle
    return: None
    -----------------------------*/
    ViewpointEvaluatorServer(ros::NodeHandlePtr node_handle);

    /*-----------------------------
    overview: Processing at the end
    argument: None
    return: None
    -----------------------------*/
    ~ViewpointEvaluatorServer();

    /*-----------------------------
    overview: Get next viewpoint(using ROS service)
    argument: req, res (Take a look at get_next_viewpoint.srv)
    return: is_succeeded - True if NBV (Next viewpoint) can be acquired normally
    Ros searvice to use: get_viewpoint_candidates
    -----------------------------*/
    bool getNBV(viewpoint_planner_3d::get_next_viewpoint::Request &req,
                viewpoint_planner_3d::get_next_viewpoint::Response &res);

    /*-----------------------------
    overview: Visualization of viewpoint candidates
    argument: None
    return: None
    -----------------------------*/
    void visualizationCandidates(void);
};
} // namespace viewpoint_evaluator_server

#endif // VIEWPOINT_EVALUATOR_SERVER_H_