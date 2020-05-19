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


class hsr_ViewpointEvaluatorState(EventState):
    '''
    Evaluate the candidate viewpoints.

    <= succeeded                        Successful evaluation of candidate viewpoints.
    <= failed                           Failure to evaluate candidate viewpoints.

    '''

    def __init__(self):
        super(hsr_ViewpointEvaluatorState, self).__init__(output_keys=["pose"], outcomes=["succeeded", "failed"])

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
