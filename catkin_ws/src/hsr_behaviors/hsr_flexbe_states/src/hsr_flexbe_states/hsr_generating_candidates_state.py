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
from viewpoint_planner_3d.srv import *


class hsr_GeneratingCandidatesState(EventState):
    '''
    Generation of viewpoint candidates.

    <= succeeded                        Successfully generated candidates.
    <= failed                           Failed to generate a candidate.

    '''

    def __init__(self, srv_name="/viewpoint_planner_3d/generating_candidates", timeout=10.0):
        super(hsr_GeneratingCandidatesState, self).__init__(outcomes=["succeeded", "failed"])
        self._srv_name = srv_name
        self._timeout = timeout

    def execute(self, userdata):
        rospy.loginfo("Generating Candidates")
        start_time = time.time()
        while self._timeout >= time.time() - start_time:
            res = self._service()
            if res.is_succeeded:
                break
            res.is_succeeded = False
        if res.is_succeeded:
            rospy.loginfo("Successfully generated candidates")
            return "succeeded"
        else:
            rospy.loginfo("Failed to generate a candidate")
            return "failed"

    def on_enter(self, userdata):
        rospy.wait_for_service(self._srv_name)
        self._service = rospy.ServiceProxy(self._srv_name, generating_candidates)

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
