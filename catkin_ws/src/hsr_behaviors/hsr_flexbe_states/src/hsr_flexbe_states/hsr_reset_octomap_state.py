#!/usr/bin/env python
#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
#
# Yusuke Miake
#
import rospy
from flexbe_core import EventState
from std_srvs.srv import Empty


class hsr_ResetOctomapState(EventState):
    '''
    Reset octomap.

    -- srv_name     String              service name

    <= succeeded                        Reset succeeded.

    '''

    def __init__(self, srv_name="/octomap_server/reset"):
        super(hsr_ResetOctomapState, self).__init__(outcomes=["succeeded"])
        self._srv_name = srv_name
        rospy.wait_for_service(self._srv_name)
        self._service = rospy.ServiceProxy(self._srv_name, Empty)

    def execute(self, userdata):
        rospy.loginfo("Reset octomap")
        self._service()
        return "succeeded"

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
