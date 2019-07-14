#!/usr/bin/env python
import rospy
import actionlib
import time
from flexbe_core import EventState, Logger
from hsrb_interface import Robot, ItemTypes

class hsr_WaitWristPressedState(EventState):
	'''
           A state to wait for a user to press downwards the wrist.
           By detecting the force on the joint, this state works as a start button.
           This code simply copies the code described in https://qa.hsr.io/ja/question/1129/sutatobotanshi-jie-da-hui/ (Japanese)

	<= succeeded				The force is detected = the wrist joint pressed
	'''

	def __init__(self):
		super(hsr_WaitWristPressedState,self).__init__(output_keys=['start_time'], outcomes=['succeeded'])
                self._robot = Robot()

	def execute(self, userdata):
                return 'succeeded'


	def on_enter(self, userdata):
            while True:
                wrench = self._robot.get('wrist_wrench',ItemTypes.FORCE_TORQUE).wrench
                if wrench[0][0] > 20.0:
                    break
            
            userdata.start_time = time.time()

	def on_exit(self, userdata):
		pass

	def on_start(self):
		pass

	def on_stop(self):
		pass
