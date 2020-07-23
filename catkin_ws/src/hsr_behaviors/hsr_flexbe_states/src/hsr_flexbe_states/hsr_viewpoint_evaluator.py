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
from geometry_msgs.msg import Pose, PoseStamped
from viewpoint_planner_3d.srv import *


class hsr_ViewpointEvaluatorState(EventState):
    '''
    Evaluate the candidate viewpoints.

    <= succeeded                        Successful evaluation of candidate viewpoints.

    '''

    def __init__(self, srv_name="/viewpoint_planner_3d/get_next_viewpoint"):
        super(hsr_ViewpointEvaluatorState, self).__init__(output_keys=["pose"], outcomes=["succeeded"])
        self._srv_name = srv_name
        rospy.wait_for_service(self._srv_name)
        self._service = rospy.ServiceProxy(self._srv_name, get_next_viewpoint)

    def execute(self, userdata):
        rospy.loginfo("Select next viewpoint")
        res = self._service()
        viewpoint_pose = res.next_viewpoint
        viewpoint_pose.position.z = res.viewpoint_height
        userdata.pose = viewpoint_pose
        print("viewpoint_pose: ")
        print(viewpoint_pose)
        return "succeeded"

    def on_enter(self, userdata):
       pass

    def on_exit(self, userdata):
        rospy.set_param("/viewpoint_planner_viewer/start_viewpoint_planning_timer", False)

    def on_start(self):
        pass

    def on_stop(self):
        pass
