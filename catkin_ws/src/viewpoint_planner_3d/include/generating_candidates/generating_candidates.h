#ifndef GENERATING_CANDIDATES_SERVER_H_
#define GENERATING_CANDIDATES_SERVER_H_

#include "viewpoint_planner_3d/generating_candidates.h"

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetMap.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/Image.h>

#include <iostream>
#include <memory>
#include <mutex>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace generating_candidates_server {
class GeneratingCandidatesServer {
  private:
    ros::NodeHandlePtr nh_;
    ros::ServiceServer gen_srv_;
    ros::Subscriber map_sub_;
    std::vector<geometry_msgs::Pose> candidates;
    nav_msgs::OccupancyGridConstPtr map_;
    double distance_between_candidates = 0.5;

    void setParam(void);
    bool generatingCandidates(viewpoint_planner_3d::generating_candidates::Request &req,
                              viewpoint_planner_3d::generating_candidates::Response &res);
    void mapCallback(const nav_msgs::OccupancyGridConstPtr &map);
    cv::Mat map2img(void);
    // costmap_2d::Costmap2D obtainCostmap();

  public:
    GeneratingCandidatesServer(ros::NodeHandlePtr node_handle);
    ~GeneratingCandidatesServer();
};
} // namespace generating_candidates_server

#endif // GENERATING_CANDIDATES_SERVER_H_