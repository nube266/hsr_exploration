#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
#
# Yusuke Miake
#

from circle import Circle
from point import Point
from segment import Segment
from dijkstra import dijkstra
from PIL import Image, ImageDraw
import math


class ObstacleMap:
    """
    Class variable name          Description
    INITIAL_POINT_VIEW_SIZE    - Initial value of point display size
    INITIAL_OBSTACLE_AREA_SIZE - Initial value of obstacle size
    """
    INITIAL_POINT_VIEW_SIZE = 8
    INITIAL_OBSTACLE_AREA_SIZE = 90

    def __init__(self):
        """
        name                  Description
        _obstacle_points      Coordinate value of obstacle
        _obstacle_area_size   The size of the area where the robot can not avoid obstacles
        _point_view_size      The value of point display size
        """
        self._obstacle_points = []
        self._obstacle_area_size = ObstacleMap.INITIAL_OBSTACLE_AREA_SIZE
        self._point_view_size = ObstacleMap.INITIAL_POINT_VIEW_SIZE

    def set_start_point(self, point):
        self._start_point = point

    def print_start_point(self):
        self._start_point.print_data()

    def set_goal_point(self, point):
        self._goal_point = point

    def print_goal_point(self):
        self._goal_point.print_data()

    def set_obstacle_area_size(self, obstacle_area_size):
        self._obstacle_area_size = obstacle_area_size

    def get_obstacle_area_size(self):
        return self._obstacle_area_size

    def set_point_view_size(self, point_view_size):
        self._point_view_size = point_view_size

    def add_obstacle_point(self, point):
        self._obstacle_points.append(point)

    def print_obstacle_point(self):
        for point in self._obstacle_points:
            point.print_data()

    def create_line_connecting_all_points(self):
        self._connect_lines = []
        all_points = self._obstacle_points[:]
        all_points.insert(0, self._start_point)
        all_points.append(self._goal_point)
        loop_count = 0
        for point1 in all_points:
            loop_count += 1
            for point2 in all_points[loop_count:]:
                if not point1.equal(point2):
                    self._connect_lines.append(Segment(point1, point2))

    def create_middle_point(self):
        self._middle_points = []
        for line in self._connect_lines:
            self._middle_points.append(line.get_middle_point())
        for point in self._obstacle_points:
            self._middle_points.append(Point(x=point.x + self._obstacle_area_size + 10,
                                             y=point.y + self._obstacle_area_size + 10))
            self._middle_points.append(Point(x=point.x + self._obstacle_area_size + 10,
                                             y=point.y - self._obstacle_area_size + 10))
            self._middle_points.append(Point(x=point.x - self._obstacle_area_size + 10,
                                             y=point.y + self._obstacle_area_size + 10))
            self._middle_points.append(Point(x=point.x - self._obstacle_area_size + 10,
                                             y=point.y - self._obstacle_area_size + 10))

    def not_overlap_line_and_obstacle_point(self, segment):
        for point in self._obstacle_points:
            if segment.get_distance_to_point(point) > self._obstacle_area_size:
                continue
            else:
                return False
        return True

    def calculate_shortest_path(self):
        self.create_route_list()
        self._shortest_path = dijkstra(self._route_list)

    def create_route_list(self):
        self._passable_lines = []
        self._route_list = []
        all_points = self._middle_points[:]
        all_points.insert(0, self._start_point)
        all_points.append(self._goal_point)
        for point1 in all_points:
            point_edge = []
            for point2 in all_points:
                if not point1.equal(point2):
                    segment = Segment(point1, point2)
                    if self.not_overlap_line_and_obstacle_point(segment):
                        point_edge.append(segment.get_length())
                        self._passable_lines.append(segment)
                    else:
                        point_edge.append(0.0)
                else:
                    point_edge.append(0.0)
            self._route_list.append(point_edge)

    def show(self, mode="FULL", window_width=1000, window_height=1000):
        """
        mode                 Description
        FULL               - Show all
        MAP_ONLY           - Show map only
        SHORTEST_PATH_ONLY - Show only the shortest path and map
        """
        img = Image.new("RGB", (window_width, window_height), (128, 128, 128))
        if self._start_point is not None:
            Circle(self._start_point, self._point_view_size).draw(
                img=img, fill=(0, 0, 255))
        if self._goal_point is not None:
            Circle(self._goal_point, self._point_view_size).draw(
                img=img, fill=(0, 255, 0))
        for point in self._obstacle_points:
            Circle(point, self._obstacle_area_size).draw(
                img=img, fill=(255, 255, 255))
            Circle(point, self._point_view_size).draw(
                img=img, fill=(0, 0, 0))
        if mode == "FULL":
            for line in self._connect_lines:
                line.draw(img=img, fill=(130, 130, 130))
            for point in self._middle_points:
                Circle(point, self._point_view_size).draw(
                    img=img, fill=(255, 255, 0))
            for line in self._passable_lines:
                line.draw(img=img, fill=(150, 0, 0))
        if mode == "FULL" or mode == "SHORTEST_PATH_ONLY":
            all_points = self._middle_points[:]
            all_points.insert(0, self._start_point)
            all_points.append(self._goal_point)
            previous_point = None
            for index in self._shortest_path:
                point = all_points[index]
                if previous_point == None:
                    previous_point = point
                    continue
                Segment(previous_point, point).draw(img=img, fill=(255, 0, 0))
                previous_point = point
        img.show()

    def create_shortest_path(self):
        self.create_line_connecting_all_points()
        self.create_middle_point()
        self.calculate_shortest_path()

    def get_shortest_path_points(self):
        all_points = self._middle_points[:]
        all_points.insert(0, self._start_point)
        all_points.append(self._goal_point)
        shortest_path_points = []
        for index in self._shortest_path:
            shortest_path_points.append(all_points[index])
        return shortest_path_points

    def print_shortest_path_point(self):
        shortest_path_points = self.get_shortest_path_points()
        for point in shortest_path_points:
            point.print_data()


if __name__ == "__main__":
    """
    Example of use
    """
    map = ObstacleMap()
    map.set_start_point(Point(500, 800))
    map.set_goal_point(Point(500, 200))
    map.add_obstacle_point(Point(x=700, y=400))
    map.add_obstacle_point(Point(x=300, y=500))
    map.add_obstacle_point(Point(x=520, y=600))
    map.add_obstacle_point(Point(x=500, y=400))
    map.create_shortest_path()
    map.print_shortest_path_point()
    map.show(mode="FULL", window_width=1000, window_height=1000)
