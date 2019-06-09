#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy
from flexbe_core.proxy import ProxyPublisher
from geometry_msgs.msg import Twist
import time

class hsr_EscapeByTwistState(EventState):
	"""
        Escapes from stucking in a collision map while navigating by move_base, by sending Twist command.
        By default, it moves backwards for 2 seconds

	-- topic 		string 			Topic to which the velocity command will be published.
        -- linear_x             float                   Linear velocity along x axis -> Forward(+) and Backward(-)
        -- linear_y             float                   Linear velocity along y axis -> Left(+) and Right(-)
        -- angular              float                   Angular velocity -> Anticlockwise(+) and Clockwise(-)
        -- duration             float                   Duration that the topic is published [sec]

	<= completed						Velcoity command has been published.
	"""
	
	def __init__(self, topic='/hsrb/command_velocity', linear_x=-0.5, linear_y=0.0, angular=0.0, duration=2.0):
		"""Constructor"""
		super(hsr_EscapeByTwistState, self).__init__(outcomes=['completed'])

		self._topic = topic
		self._pub = ProxyPublisher({self._topic: Twist})

                # Twist data to be published 
                twist = Twist()
                twist.linear.x = linear_x
                twist.linear.y = linear_y
                twist.angular.z = angular

                self._twist = twist

                self._duration = duration


	def execute(self, userdata):
		return 'completed'
	
	def on_enter(self, userdata):
            #
            # Publish the twist data within the given duration
            #
            timeout = time.time() + self._duration
            while True:
		self._pub.publish(self._topic, self._twist)

                if time.time() > timeout:
                    break
