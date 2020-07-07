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
import numpy as np
import sys
from scipy.sparse.csgraph import dijkstra
from scipy.sparse import csr_matrix


class dijkstra_server:

    def __init__(self):
        pass

    def get_shortest_path_length(self, req):
        node1 = list(req.node1)
        node2 = list(req.node2)
        one_matrix = [1] * len(node1)
        node1_max = max(node1)
        node2_max = max(node2)
        if node1_max >= node2_max:
            csr = csr_matrix((one_matrix, (node1, node2)), shape=(node1_max + 1, node1_max + 1))
        else:
            csr = csr_matrix((one_matrix, (node1, node2)), shape=(node2_max + 1, node2_max + 1))
        dist = dijkstra(csr, unweighted=True, indices=req.start_node).tolist()
        temp = np.array(dist)
        np.place(temp, temp == float('inf'), 2147483646)
        distances = temp.tolist()
        return get_shortest_path_lengthResponse(distances)


if __name__ == "__main__":
    rospy.init_node("dijkstra_server")
    node = dijkstra_server()
    srv = rospy.Service("/dijkstra_server/get_shortest_path_length",
                        get_shortest_path_length, node.get_shortest_path_length)
    rospy.loginfo("Ready to dijkstra_server")
    rospy.spin()
