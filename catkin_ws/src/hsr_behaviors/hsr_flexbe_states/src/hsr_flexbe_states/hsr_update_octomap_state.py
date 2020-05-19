#!/usr/bin/env python
#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
#
# Yusuke Miake
#
import rospy
import time
from flexbe_core import EventState
from flexbe_core import Logger
from flexbe_core.proxy import ProxyServiceCaller
from octomap_publisher.srv import *


class hsr_UpdateOctomapState(EventState):
    '''
    State to update octomap.

    -- srv_name     String              Service name
    -- timeout      float               Octomap update timeout period

    <= succeeded                        Update was succeeded.
    <= failed                           Update was failed.

    '''

    def __init__(self, srv_name="/octomap_publisher/update_octomap", timeout=10.0):
        super(hsr_UpdateOctomapState, self).__init__(outcomes=["succeeded", "failed"])
        self._srv_name = srv_name
        self._timeout = timeout

    def execute(self, userdata):
        rospy.loginfo("Update octomap")
        start_time = time.time()
        while self._timeout >= time.time() - start_time:
            res = self._service()
            if res.is_succeeded:
                break
        if res.is_succeeded:
            rospy.loginfo("Octomap updated successfully")
            return "succeeded"
        else:
            rospy.loginfo("Octomap update failed")
            return "failed"

    def on_enter(self, userdata):
        rospy.wait_for_service(self._srv_name)
        self._service = rospy.ServiceProxy(self._srv_name, update_octomap)

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
