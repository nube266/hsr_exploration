#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_wait_press_wrist_state import hsr_WaitWristPressedState
from hsr_flexbe_states.hsr_speak_state import hsr_SpeakState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jun 26 2019
@author: ShigemichiMatsuzaki
'''
class HSRTestStartButtonSM(Behavior):
	'''
	Test code of hsr_WaitWristPressedState
	'''


	def __init__(self):
		super(HSRTestStartButtonSM, self).__init__()
		self.name = 'HSR Test Start Button'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Start',
										hsr_WaitWristPressedState(),
										transitions={'succeeded': 'Speak'},
										autonomy={'succeeded': Autonomy.Off})

			# x:136 y:189
			OperatableStateMachine.add('Speak',
										hsr_SpeakState(sentence='The button pressed', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
