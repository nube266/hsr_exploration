#!/usr/bin/env python
#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
#
# Yusuke Miake
#
import rospy
from flexbe_core import EventState
import subprocess
import time
import threading


class hsr_RestartGmappingState(EventState):
    '''
    Restart gmapping.

    <= succeeded                        Reset succeeded.

    '''

    def __init__(self):
        super(hsr_RestartGmappingState, self).__init__(outcomes=["succeeded"])

    def execute(self, userdata):
        rospy.loginfo("Restart gmapping")
        time.sleep(2)
        thread = threading.Thread(target=self.launch_gmapping)
        thread.start()
        rospy.wait_for_service("/gmapping/get_loggers")
        rospy.wait_for_service("/move_base/get_loggers")
        return "succeeded"

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

    def launch_gmapping(self):
        cmd = "roslaunch hsrb_mapping hsrb_mapping.launch"
        subprocess.call(cmd.split())
