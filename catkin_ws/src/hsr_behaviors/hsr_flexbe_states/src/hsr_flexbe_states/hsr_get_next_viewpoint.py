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
from viewpoint_planner.srv import *


class hsr_GetNextViewpoint(EventState):
    '''
    The state from which to get the next viewpoint

    -- srv_name     String              service name

    <= succeeded                        Succeeded to get next viewpoint
    <= None                             There is no place to go next viewpoint
    '''

    def __init__(self, srv_name="/viewpoint_planner/get_next_viewpoint"):
        super(hsr_GetNextViewpoint, self).__init__(output_keys=["pose"], outcomes=["succeeded", "failed"])
        self._srv_name = srv_name

    def execute(self, userdata):
        rospy.loginfo("Get newxt viewpoint")
        res = self._service()
        if res.is_succeeded:
            rospy.loginfo("Succeeded to get next viewpoint")
            userdata.pose = res.next_viewpoint
            return "succeeded"
        else:
            rospy.loginfo("There is no place to go next viewpoint")
            userdata.pose = res.next_viewpoint
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
