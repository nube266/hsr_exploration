#!/usr/bin/env python
#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
#
# Yusuke Miake
#
import rospy
from flexbe_core import EventState
from flexbe_core import Logger
from flexbe_core.proxy import ProxyServiceCaller

class hsr_StartTimerState(EventState):
    '''
    State to start timer
    viewpoint_planner_viewer node required

    -- timer_param_name     String      Parameter name of the timer to start(See viewpoint_planner_viewer.py)

    '''

    def __init__(self, param_name="/viewpoint_planner_viewer/start_total_search_timer"):
        super(hsr_StartTimerState, self).__init__(outcomes=["succeeded"])
        self._param_name = param_name

    def execute(self, userdata):
        print("Start timer")
        rospy.set_param(self._param_name, True)
        return "succeeded"

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
