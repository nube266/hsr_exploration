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
    ros::Subscriber odom_sub_;              // ROS subscriber to get the current robot position
    ros::Subscriber octomap_sub_;           // ROS subscriber to get the Octomap(Octree)
    ros::ServiceClient make_path_cli_;      // move_base route planning service client
    ros::ServiceClient clear_costmaps_cli_; // Delete move_base cost map

    /* Variables used to evaluate viewpoint candidates */
    geometry_msgs::Pose current_robot_pose_;     // Current robot pose
    std::vector<geometry_msgs::Pose> candidates; // Viewpoint candidates
    std::vector<double> distances;               // Distance to each viewpoint candidates
    navfn::NavfnROS planner_;                    // Path planner

    /* Octomap */
    octomap::OcTree *octree_ = nullptr;
    std::mutex octree_mutex;
    std::chrono::system_clock::time_point current_get_octree_time;  // Time to get the latest Octree
    std::chrono::system_clock::time_point previous_get_octree_time; // 前回視点計画

    /* Parameter */
    double timeout = 10.0;                  // Timeout time when stopped by some processing[s]
    double candidate_marker_lifetime = 5.0; //Display time when the marker is visualized
    double path_planning_tolerance = 0.3;   // Path planning tolerance
    std::string odom_topic = "/hsrb/odom";  // Odometry topic name

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
    overview: Returns the total travel distance in the entered travel plan
    argument: plan(toravel plan)
    return: distance[m]
    -----------------------------*/
    double calcTravelDistance(const nav_msgs::Path &plan);

    /*-----------------------------
    overview: Return travel planning results using move_base's ROS service
    argument: Start point of movement, end point of movement, travel planning
    return: travel planning, returns true if route planning succeeds
    -----------------------------*/
    bool makePathPlan(const geometry_msgs::Pose &start, const geometry_msgs::Pose &goal, nav_msgs::Path &plan);

    /*-----------------------------
    overview: Calculate the distance traveled to each viewpoint candidate and set it in 'distances'
    argument: None
    return: None
    set: distances
    -----------------------------*/
    void calcViewpointDistances(void);

    /*-----------------------------
    overview: Get viewpoint from 'generating_candidates' by service call
    argument: None
    return: None
    set: candidates
    using: get_candidates_cli_(generating_candidates)
    -----------------------------*/
    bool getCandidates(void);

    /*-----------------------------
    overview: Evaluate viewpoint candidates
    argument: None
    return: Returns true if the viewpoint candidate is evaluated successfully
    using: candidates, distances
    -----------------------------*/
    bool evaluateViewpoints(void);

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