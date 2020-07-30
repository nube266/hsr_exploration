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
from std_srvs.srv import Empty

class hsr_RestartGmappingState(EventState):
    '''
    Restart gmapping.

    <= succeeded                        Reset succeeded.

    '''

    def __init__(self):
        super(hsr_RestartGmappingState, self).__init__(outcomes=["succeeded"])
        rospy.wait_for_service("/move_base/clear_costmaps")
        self._clear_costmap_srv = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)

    def execute(self, userdata):
        rospy.loginfo("Restart gmapping")
        time.sleep(2)
        thread = threading.Thread(target=self.launch_gmapping)
        thread.start()
        time.sleep(5)
        self._clear_costmap_srv()
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
        cmd = "rosnode kill /gmapping "
        subprocess.call(cmd.split())
        time.sleep(2)
        cmd = "roslaunch hsrb_mapping gmapping.launch"
        subprocess.call(cmd.split())
