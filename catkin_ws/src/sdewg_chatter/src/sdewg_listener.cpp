#include "ros/ros.h"
#include "std_msgs/String.h"

// http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Received: %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sdewg_listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("sdewg_chatter", 1000, chatterCallback);
  ros::spin();
  return 0;
}
