#!/usr/bin/env python
import rospy
import time
from flexbe_core import EventState, Logger

class hsr_SetStartTimeState(EventState):
	'''
        A state to set the start time of the task in order to later check the elapsed time with 'hsr_CheckElapsedTimeState'.

	<= succeeded				The force is detected = the wrist joint pressed
	'''

	def __init__(self):
		super(hsr_SetStartTimeState,self).__init__(output_keys=['start_time'], outcomes=['succeeded'])

	def execute(self, userdata):
                return 'succeeded'

	def on_enter(self, userdata):
            userdata.start_time = time.time()
