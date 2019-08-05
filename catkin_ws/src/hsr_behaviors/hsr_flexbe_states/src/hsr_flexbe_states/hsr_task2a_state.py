#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from avoidance_server.srv import *
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller


class hsr_Task2aState(EventState):
    '''
    Move the HSR base to the 'goal' target pose using an Actionlib action.

    ># pose             Pose	Goal pose that the base should reach.

    -- threshold_x_max  float   Threshold of point that robot can reach (Relative distance from work start point [m])
    -- threshold_y_max  float   Threshold of point that robot can reach (Relative distance from work start point [m])

    <= succeeded			    The base has succesfully moved to the goal pose.
    <= failed				    The base could not move to the goal pose.
    '''

    def __init__(self, move_srv_name="/avoidance_move_server/move", threshold_x_max=2.0, threshold_y_max=2.0):
        super(hsr_Task2aState, self).__init__(outcomes=['succeeded', 'failed'],
                                              input_keys=['pose'])
        self._move_srv_name = move_srv_name
        self._move_server = ProxyServiceCaller({self._move_srv_name: avoidance_move_server})
        self._threshold_x_max = threshold_x_max
        self._threshold_y_max = threshold_y_max

    def execute(self, userdata):
        rospy.loginfo('Let\'s move')
        rospy.sleep(1)
        # If succeeded to move, return 'succeeded'. Otherwise, 'failed'.
        if not self._failed:
            return 'succeeded'
        else:
            return 'failed'

    def on_enter(self, userdata):
        goal_point_x = userdata.pose.position.x
        goal_point_y = userdata.pose.position.y
        req = avoidance_move_serverRequest(goal_point_x, goal_point_y, self._threshold_x_max, self._threshold_y_max)
        self._failed = False
        try:
            self._srv_result = self._move_server.call(self._move_srv_name, req)
            rospy.loginfo(self._srv_result)
            self._failed = not self._srv_result.is_succeeded
        except Exception as e:
            rospy.logwarn('Failed to call object recognizer:\n\r%s' % str(e))
            self._failed = True

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
