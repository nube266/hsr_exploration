#!/usr/bin/python
#h -*- coding: utf-8 -*-

#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
# 
# Yusuke Miake
#


import rospy
from dijkstra_path_server.srv import  path_server, path_serverResponse

def handle_add_two_ints(req, res):
    print(req.obstacle_point_x)

def add_two_ints_server():
    rospy.init_node('path_server')
    s = rospy.Service('path_server', path_server, handle_add_two_ints)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()