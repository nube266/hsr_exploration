#!/usr/bin/env python
#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
#
# Yusuke Miake
#
import rospy
from flexbe_core import EventState


class hsr_ResetViewerState(EventState):
    '''
    Reset viewer.

    -- 

    <= succeeded                        Reset succeeded.

    '''

    def __init__(self):
        super(hsr_ResetViewerState, self).__init__(outcomes=["succeeded"])

    def execute(self, userdata):
        rospy.loginfo("Reset viewer")
        rospy.set_param("/viewpoint_planner_viewer/reset", True)
        return "succeeded"

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
