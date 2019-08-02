#!/usr/bin/python
# h -*- coding: utf-8 -*-

#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
#
# Yusuke Miake
#

from avoidance_server.srv import *
from dijkstra_path_server.srv import *
import hsrb_interface
import rospy


class move_server:

    def __init__(self):
        pass

    def set_robot_point(self):
        self.robot_point_x = 5.0 * 100
        self.robot_point_y = 8.0 * 100

    def set_obstacle_points(self):
        self.obstacle_points_x = []
        self.obstacle_points_y = []
        self.obstacle_points_x.append(7.0 * 100)
        self.obstacle_points_x.append(3.0 * 100)
        self.obstacle_points_x.append(5.2 * 100)
        self.obstacle_points_x.append(7.0 * 100)
        self.obstacle_points_y.append(4.0 * 100)
        self.obstacle_points_y.append(5.0 * 100)
        self.obstacle_points_y.append(6.0 * 100)
        self.obstacle_points_y.append(4.0 * 100)

    def set_goal_point(self):
        self.goal_point_x = 5.0 * 100
        self.goal_point_y = 2.0 * 100

    def print_points(self):
        print("---------------------")
        print("robot:")
        print("x:\t{0}\ty:\t{1} [cm]".format(self.robot_point_x,
                                             self.robot_point_y))
        print("goal:")
        print("x:\t{0}\ty:\t{1} [cm]".format(self.goal_point_x,
                                             self.goal_point_y))
        for i in range(len(self.obstacle_points_x)):
            print("obstacle point {0}:".format(i))
            print("x:\t{0}\ty:\t{1} [cm]".format(self.obstacle_points_x[i],
                                                 self.obstacle_points_y[i]))
        for i in range(len(self.shortest_path_point_x)):
            print("shortest_path_point {0}:".format(i))
            print("x:\t{0}\ty:\t{1} [cm]".format(self.shortest_path_point_x[i],
                                                 self.shortest_path_point_y[i]))
        print("---------------------")

    def calculate_shortest_path(self):
        rospy.wait_for_service("/dijkstra_path_server/get_path", 10.0)
        try:
            dijkstra_path = rospy.ServiceProxy("/dijkstra_path_server/get_path",
                                               dijkstra_path_server)
            res = dijkstra_path(self.robot_point_x,
                                self.robot_point_y,
                                self.goal_point_x,
                                self.goal_point_y,
                                self.obstacle_points_x,
                                self.obstacle_points_y)
            self.shortest_path_point_x = res.shortest_path_point_x
            self.shortest_path_point_y = res.shortest_path_point_y
        except Exception as e:
            rospy.logerr(e)

    def avoidance_move(self, req):
        print("Start /avoidance_move_server/move")
        self.set_robot_point()
        self.set_goal_point()
        self.set_obstacle_points()
        self.calculate_shortest_path()
        self.print_points()
        return avoidance_move_serverResponse(True)


if __name__ == "__main__":
    rospy.init_node("avoidance_move_server")
    server = move_server()
    srv = rospy.Service("/avoidance_move_server/move",
                        avoidance_move_server,
                        server.avoidance_move)
    print("Ready to move_server")
    rospy.spin()
