#ifndef VIEWPOINT_EVALUATOR_SERVER_H_
#define VIEWPOINT_EVALUATOR_SERVER_H_

// ros searvice
#include "viewpoint_planner_3d/get_next_viewpoint.h"

// ros
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetMap.h>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

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
    ros::NodeHandlePtr nh_;          // ROS node handle
    ros::ServiceServer get_nbv_srv_; // ROS service that gets viewpoint candidates

    /* Variables for occupied grid map */
    std::vector<geometry_msgs::Pose> candidates; // Viewpoint candidate
    double timeout = 10.0;                       // Timeout time when stopped by some processing[s]

    /*-----------------------------
    overview: Set of ROS parameters
    argument: None
    Return: None
    -----------------------------*/
    void setParam(void);

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
};
} // namespace viewpoint_evaluator_server

#endif // VIEWPOINT_EVALUATOR_SERVER_H_