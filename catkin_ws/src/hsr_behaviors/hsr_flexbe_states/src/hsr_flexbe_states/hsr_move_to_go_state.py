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
import hsrb_interface


class hsr_MoveToGoState(EventState):

    def __init__(self):
        super(hsr_MoveToGoState, self).__init__(outcomes=["succeeded", "failed"])
        self._robot = hsrb_interface.Robot()
        self._whole_body = self._robot.get('whole_body')

    def execute(self, userdata):
        try:
            self._whole_body.move_to_go()
            is_succeeded = True
        except Exception as e:
            rospy.logerr(e)
            is_succeeded = False
        rospy.sleep(1)
        if is_succeeded:
            return "succeeded"
        else:
            return "failed"

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
