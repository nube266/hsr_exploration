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
from hsr_flexbe_states.hsr_check_elapsed_time_state import hsr_CheckElapsedTimeState
from hsr_flexbe_states.hsr_speak_state import hsr_SpeakState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Jul 14 2019
@author: ShigemichiMatsuzaki
'''
class HSRTestTimerSM(Behavior):
	'''
	Test behavior to check 'hsr_CheckElapsedTimeState'
	'''


	def __init__(self):
		super(HSRTestTimerSM, self).__init__()
		self.name = 'HSR Test Timer'

		# parameters of this behavior
		self.add_parameter('time_limit', 1.0)

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
										transitions={'succeeded': 'CheckTime'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'start_time': 'start_time'})

			# x:71 y:139
			OperatableStateMachine.add('CheckTime',
										hsr_CheckElapsedTimeState(time_limit=self.time_limit),
										transitions={'time_remains': 'SpeakTimeRemains', 'time_up': 'SpeakTimeUp'},
										autonomy={'time_remains': Autonomy.Off, 'time_up': Autonomy.Off},
										remapping={'start_time': 'start_time'})

			# x:221 y:241
			OperatableStateMachine.add('SpeakTimeUp',
										hsr_SpeakState(sentence='Time is up', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:312 y:111
			OperatableStateMachine.add('SpeakTimeRemains',
										hsr_SpeakState(sentence='Time still remains. Keep it up', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'Wait'},
										autonomy={'done': Autonomy.Off})

			# x:416 y:192
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=5.0),
										transitions={'done': 'CheckTime'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
