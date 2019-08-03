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
import actionlib
from geometry_msgs.msg import Pose, PoseStamped
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class move_server:

    def __init__(self):
        self._robot = hsrb_interface.Robot()
        self._omni_base = self._robot.get('omni_base')
        self._cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

    def set_robot_point(self):
        self._robot_point_x = self._omni_base.pose[0] * 100
        self._robot_point_y = self._omni_base.pose[1] * 100

    def set_obstacle_points(self):
        self._obstacle_points_x = []
        self._obstacle_points_y = []
        rospy.wait_for_service("/avoidance_server/get_obstacle_points", 10.0)
        try:
            get_obstacle_points_srv = rospy.ServiceProxy("/avoidance_server/get_obstacle_points",
                                                         avoidance_server)
            res = get_obstacle_points_srv()
            self._obstacle_points_x = [n * 100 for n in res.obstacle_points_x]
            self._obstacle_points_y = [n * 100 for n in res.obstacle_points_y]
        except Exception as e:
            rospy.logerr(e)

    def set_goal_point(self, req):
        self._goal_point_x = req.goal_point_x * 100
        self._goal_point_y = req.goal_point_y * 100

    def print_points(self):
        print("---------------------")
        print("robot:")
        print("x:\t{0}\ty:\t{1} [cm]".format(self._robot_point_x,
                                             self._robot_point_y))
        print("goal:")
        print("x:\t{0}\ty:\t{1} [cm]".format(self._goal_point_x,
                                             self._goal_point_y))
        for i in range(len(self._obstacle_points_x)):
            print("obstacle point {0}:".format(i))
            print("x:\t{0}\ty:\t{1} [cm]".format(self._obstacle_points_x[i],
                                                 self._obstacle_points_y[i]))
        for i in range(len(self._shortest_path_point_x)):
            print("shortest_path_point {0}:".format(i))
            print("x:\t{0}\ty:\t{1} [cm]".format(self._shortest_path_point_x[i],
                                                 self._shortest_path_point_y[i]))
        print("---------------------")

    def calculate_shortest_path(self):
        rospy.wait_for_service("/dijkstra_path_server/get_path", 10.0)
        try:
            dijkstra_path = rospy.ServiceProxy("/dijkstra_path_server/get_path",
                                               dijkstra_path_server)
            res = dijkstra_path(self._robot_point_x,
                                self._robot_point_y,
                                self._goal_point_x,
                                self._goal_point_y,
                                self._obstacle_points_x,
                                self._obstacle_points_y)
            self._shortest_path_point_x = res.shortest_path_point_x
            self._shortest_path_point_y = res.shortest_path_point_y
            print(res.is_succeeded)
        except Exception as e:
            rospy.logerr(e)

    def avoidance_move(self, req):
        print("Start /avoidance_move_server/move")
        self.set_robot_point()
        self.set_goal_point(req)
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
