#!/usr/bin/python
# -*- coding: utf-8 -*-

#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
#
# Yusuke Miake
#

from viewpoint_planner_3d.srv import get_shortest_path_length, get_shortest_path_lengthResponse
import rospy


class dijkstra_server:

    def __init__(self):
        pass

    def get_shortest_path_length(self, req):
        distances = []
        return get_shortest_path_lengthResponse(distances)


if __name__ == "__main__":
    rospy.init_node("dijkstra_server")
    node = dijkstra_server()
    srv = rospy.Service("/dijkstra_server/get_shortest_path_length",
                        get_shortest_path_length, node.get_shortest_path_length)
    rospy.loginfo("Ready to dijkstra_server")
    rospy.spin()
