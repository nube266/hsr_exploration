#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

// http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sdewg_talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("sdewg_chatter", 1000);
  ros::Rate loop_rate(10);

  int count = 0;

  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;

    ss << "Hello world! " << count;
    msg.data = ss.str();
    ROS_INFO("Sent: %s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
