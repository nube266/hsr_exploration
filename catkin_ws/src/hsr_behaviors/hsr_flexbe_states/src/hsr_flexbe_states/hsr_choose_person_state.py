#!/usr/bin/env python
import rospy
import time
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core import EventState, Logger
from geometry_msgs.msg import Point
import hsrb_interface

class hsr_ChoosePersonState(EventState):
	'''
        A state to refer to the result of person tracking, and return which side the person is.

        -- topic_name                   The name of the topic of person tracking result (geometry_msgs/Point)

	<= left				The person is on the left
	<= right			The person is on the right
	'''

	def __init__(self, topic_name='/monocular_person_following/target_pose'):
		super(hsr_ChoosePersonState,self).__init__(outcomes=['left', 'right'])
                self._sub = ProxySubscriberCached({topic_name: Point})
                self._topic = topic_name

                self._robot = hsrb_interface.Robot()
                self._whole_body = self._robot.get('whole_body')

	def execute(self, userdata):
                if self._sub.has_msg(self._topic):
                    msg = self._sub.get_last_msg(self._topic)
                    # in case you want to make sure the same message is not processed twice:
                    self._sub.remove_last_msg(self._topic) 

                    # If the y value of the estimated person position (in base_link frame) is positive,
                    #   the person is on the left. Otherwise on the right.
                    if msg.y > 0:
                        return 'left'
                    else:
                        return 'right'

	def on_enter(self, userdata):
                # Sleep to wait for the person detection
                self._whole_body.move_to_joint_positions({
                    'arm_roll_joint': 1.5,
                    'head_tilt_joint': -0.178
                })

                rospy.sleep(5)

        def on_exit(self, userdata):
                # Sleep to wait for the person detection
                self._whole_body.move_to_joint_positions({
                    'arm_roll_joint': 0.0
                })

