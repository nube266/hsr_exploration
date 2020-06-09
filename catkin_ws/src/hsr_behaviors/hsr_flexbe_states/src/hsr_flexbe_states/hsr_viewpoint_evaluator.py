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
from viewpoint_planner_3d.srv import *


class hsr_ViewpointEvaluatorState(EventState):
    '''
    Evaluate the candidate viewpoints.

    <= succeeded                        Successful evaluation of candidate viewpoints.
    <= failed                           Failure to evaluate candidate viewpoints.

    '''

    def __init__(self, srv_name="/viewpoint_planner_3d/get_next_viewpoint"):
        super(hsr_ViewpointEvaluatorState, self).__init__(output_keys=["pose"], outcomes=["succeeded", "failed"])
        self._srv_name = srv_name

    def execute(self, userdata):
        rospy.loginfo("Select next viewpoint")
        res = self._service()
        if res.is_succeeded:
            rospy.loginfo("Successfully selected next viewpoint")
            return "succeeded"
        else:
            rospy.loginfo("Failed to select next viewpoint")
            return "failed"

    def on_enter(self, userdata):
        rospy.wait_for_service(self._srv_name)
        self._service = rospy.ServiceProxy(self._srv_name, get_next_viewpoint)

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
