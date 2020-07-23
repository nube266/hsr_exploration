#!/usr/bin/python
# h -*- coding: utf-8 -*-

#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
#
# Yusuke Miake
#
from Tkinter import *
import rospy
import time


class ViewpointPlannerViewerNode:
    def __init__(self):
        rospy.init_node("viewpoint_planner_viewer")
        self.setup_window()

    def setup_window(self):
        self._root = Tk()
        self._root.title("viewpoint planner viewer")
        self._root.geometry("1100x200")
        self.initialization_total_search_time_label()
        self.initialization_total_viewpoint_planning_time_label()
        self.initialization_number_of_moves()
        self.initialization_total_travel_time()

    def main_loop(self):
        while not rospy.is_shutdown():
            self.update_total_search_time()
            self.update_total_viewpoint_planning_time()
            self.update_number_of_moves()
            self.update_total_travel_time()
            self._root.update_idletasks()

    def set_label(self, x, y, text):
        var = StringVar()
        var.set(text)
        lbl = Label(self._root, textvariable=var, font=("", 30))
        lbl.place(x=x, y=y)

    def initialization_total_search_time_label(self):
        self.set_label(0, 0, "TOTAL SEARCH TIME")
        self._total_search_time = 0.0
        self._start_total_search_timer_param = "/viewpoint_planner_viewer/start_total_search_timer"
        rospy.set_param(self._start_total_search_timer_param, False)
        self._last_update_total_search_time = time.time()
        self.set_label(770, 0, "{:.2f} [sec]".format(self._total_search_time))

    def initialization_total_viewpoint_planning_time_label(self):
        self.set_label(0, 45, "TOTAL VIEWPOINT PLANNING TIME")
        self._total_viewpoint_planning_time = 0.0
        self._start_viewpoint_planning_timer_param = "/viewpoint_planner_viewer/start_viewpoint_planning_timer"
        rospy.set_param(self._start_viewpoint_planning_timer_param, False)
        self._last_update_total_viewpoint_planning_time = time.time()
        self.set_label(770, 45, "{:.2f} [sec]".format(self._total_viewpoint_planning_time))

    def initialization_number_of_moves(self):
        self.set_label(0, 90, "NUMBER OF MOVES")
        self._number_of_moves = 0
        self._add_number_of_moves_param = "/viewpoint_planner_viewer/add_number_of_moves"
        rospy.set_param(self._add_number_of_moves_param, 0)
        self.set_label(770, 90, "{:.0f}".format(self._number_of_moves))

    def initialization_total_travel_time(self):
        self.set_label(0, 135, "TOTAL TRAVEL TIME")
        self._total_travel_time = 0.0
        self._start_total_travel_timer_param = "/viewpoint_planner_viewer/start_total_travel_timer"
        rospy.set_param(self._start_total_travel_timer_param, False)
        self._last_update_total_travel_time = time.time()
        self.set_label(770, 135, "{:.2f} [m]".format(self._total_travel_time))

    def update_total_search_time(self):
        if rospy.get_param(self._start_total_search_timer_param, False) == True:
            self._total_search_time += time.time() - self._last_update_total_search_time
            self.set_label(770, 0, "{:.2f} [sec]".format(self._total_search_time))
        self._last_update_total_search_time = time.time()

    def update_total_viewpoint_planning_time(self):
        if rospy.get_param(self._start_viewpoint_planning_timer_param, False) == True:
            self._total_viewpoint_planning_time += time.time() - self._last_update_total_viewpoint_planning_time
            self.set_label(770, 45, "{:.2f} [sec]".format(self._total_viewpoint_planning_time))
        self._last_update_total_viewpoint_planning_time = time.time()

    def update_number_of_moves(self):
        if int(rospy.get_param(self._add_number_of_moves_param, 0)) == 0:
            return
        self._number_of_moves += int(rospy.get_param(self._add_number_of_moves_param, 0))
        rospy.set_param(self._add_number_of_moves_param, 0.0)
        self.set_label(770, 90, "{:.0f}".format(self._number_of_moves))

    def update_total_travel_time(self):
        if rospy.get_param(self._start_total_travel_timer_param, False) == True:
            self._total_travel_time += time.time() - self._last_update_total_travel_time
            self.set_label(770, 135, "{:.2f} [m]".format(self._total_travel_time))
        self._last_update_total_travel_time = time.time()


if __name__ == "__main__":
    node = ViewpointPlannerViewerNode()
    node.main_loop()
