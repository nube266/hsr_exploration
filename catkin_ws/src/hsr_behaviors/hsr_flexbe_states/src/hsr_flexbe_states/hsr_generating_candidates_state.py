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


class hsr_GeneratingCandidatesState(EventState):
    '''
    Generation of viewpoint candidates.

    <= succeeded                        Successfully generated candidates.
    <= failed                           Failed to generate a candidate.

    '''

    def __init__(self):
        super(hsr_GeneratingCandidatesState, self).__init__(outcomes=["succeeded", "failed"])

    def execute(self, userdata):
        pass

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
