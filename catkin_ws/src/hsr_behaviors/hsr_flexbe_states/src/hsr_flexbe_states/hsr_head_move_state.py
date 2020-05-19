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
import hsrb_interface


class hsr_HeadMoveState(EventState):

    def __init__(self, head_position=0.0):
        super(hsr_HeadMoveState, self).__init__(outcomes=["succeeded"])
        self._head_position = head_position
        self._robot = hsrb_interface.Robot()
        self._whole_body = self._robot.get("whole_body")

    def execute(self, userdata):
        self._whole_body.move_to_joint_positions({"head_tilt_joint": self._head_position})
        return "succeeded"

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
