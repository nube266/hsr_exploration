#!/usr/bin/env python
#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
#
# Yusuke Miake
#
import rospy
import os
from flexbe_core import EventState
import datetime


class hsr_SaveViewerDataState(EventState):
    '''
    Save viewwer data(Save viewpoiint_planner_viweer node)

    -- save_path    String          Where to save

    <= succeeded                    Save succeeded.

    '''

    def __init__(self, save_path="/root/HSR/catkin_ws/src/hsr_behaviors/hsr_flexbe_states/src/hsr_flexbe_states/data/log.csv"):
        super(hsr_SaveViewerDataState, self).__init__(outcomes=["succeeded"])
        self._save_path = save_path
        if os.path.isfile(self._save_path) is False:
            with open(self._save_path, mode="w") as f:
                f.write("date,total_search_time,total_viewpoint_planning_time,number_of_moves,total_travel_time\n")

    def execute(self, userdata):
        total_search_time = rospy.get_param("/viewpoint_planner_viewer/total_search_time", 0.0)
        total_viewpoint_planning_time = rospy.get_param("/viewpoint_planner_viewer/total_viewpoint_planning_time", 0.0)
        number_of_moves = rospy.get_param("/viewpoint_planner_viewer/number_of_moves", 0.0)
        total_travel_time = rospy.get_param("/viewpoint_planner_viewer/total_travel_time", 0.0)
        print("---------------------")
        print("Save data")
        print("total_search_time: {0}".format(total_search_time))
        print("total_viewpoint_planning_time: {0}".format(total_viewpoint_planning_time))
        print("number_of_moves: {0}".format(number_of_moves))
        print("total_travel_time: {0}".format(total_travel_time))
        print("---------------------")
        with open(self._save_path, mode="a") as f:
            dt_now = datetime.datetime.now()
            f.write("{0},{1},{2},{3},{4}\n".format(dt_now.strftime("%Y-%m-%d-%H-%M-%S"), total_search_time, total_viewpoint_planning_time,
                                                   number_of_moves, total_travel_time))
        return "succeeded"

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
