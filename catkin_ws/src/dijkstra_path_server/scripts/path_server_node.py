#!/usr/bin/python
# -*- coding: utf-8 -*-

#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
#
# Yusuke Miake
#

from dijkstra_path_server.srv import dijkstra_path_server, dijkstra_path_serverResponse
from obstacle_map import ObstacleMap
from point import Point
import rospy


class path_server:

    def __init__(self):
        self._is_succeeded = False

    def create_map(self, req):
        self._map = ObstacleMap()
        self._map.set_start_point(Point(x=req.start_point_x,
                                        y=req.start_point_y))
        print("start_point:")
        self._map.print_start_point()
        self._map.set_goal_point(Point(x=req.goal_point_x,
                                       y=req.goal_point_y))
        print("goal_point:")
        self._map.print_goal_point()
        print("obstacle_point:")
        for i in range(len(req.obstacle_point_x)):
            self._map.add_obstacle_point(Point(x=req.obstacle_point_x[i],
                                               y=req.obstacle_point_y[i]))
        self._map.print_obstacle_point()

    def calculate_short_path(self):
        self._map.create_shortest_path()
        print("shortest_path")
        self._map.print_shortest_path_point()
        shortest_paht_points = self._map.get_shortest_path_points()
        self._shortest_path_point_x = []
        self._shortest_path_point_y = []
        for point in shortest_paht_points:
            self._shortest_path_point_x.append(point.x)
            self._shortest_path_point_y.append(point.y)
        if len(shortest_paht_points) >= 2:  # can't create short path
            self._is_succeeded = True
        else:
            self._is_succeeded = False

    def get_path(self, req):
        rospy.loginfo("Calculate shortest path")
        self.create_map(req)
        print(self._map.get_obstacle_area_size())
        while not self._is_succeeded and self._map.get_obstacle_area_size() > 0:
            self.calculate_short_path()
            self._map.set_obstacle_area_size(self._map.get_obstacle_area_size() - 5)
        print(self._map.get_obstacle_area_size())
        print(self._is_succeeded)
        return dijkstra_path_serverResponse(self._shortest_path_point_x,
                                            self._shortest_path_point_y,
                                            self._is_succeeded)


if __name__ == "__main__":
    rospy.init_node("dijkstra_path_server")
    p = path_server()
    srv = rospy.Service("/dijkstra_path_server/get_path",
                        dijkstra_path_server, p.get_path)
    rospy.loginfo("Ready to dijkstra_path_server")
    rospy.spin()
