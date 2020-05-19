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
import subprocess


class hsr_ResetOctomapState(EventState):
    '''
    Reset octomap.

    -- srv_name     String              service name

    <= succeeded                        Reset succeeded.

    '''

    def __init__(self, srv_name="/octomap_server/reset"):
        super(hsr_ResetOctomapState, self).__init__(outcomes=["succeeded"])
        self._srv_name = srv_name

    def execute(self, userdata):
        rospy.loginfo("Reset octomap")
        cmd = "rosservice call " + self._srv_name
        subprocess.call(cmd.split())
        return "succeeded"

    def on_enter(self, userdata):
        rospy.wait_for_service(self._srv_name)

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
