#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_speak_state import hsr_SpeakState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jun 11 2019
@author: ShigemichiMatsuzaki
'''
class HSRSpeakStateTestSM(Behavior):
	'''
	Demo code of hsr_SpeakState
	'''


	def __init__(self):
		super(HSRSpeakStateTestSM, self).__init__()
		self.name = 'HSR Speak State Test'

		# parameters of this behavior
		self.add_parameter('searching_point', 'searching_point_0')
		self.add_parameter('putting_point', 'rubbishbin')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:87 y:660, x:805 y:508
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:141 y:149
			OperatableStateMachine.add('Speak',
										hsr_SpeakState(sentence='Fuck you Inouchi', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
