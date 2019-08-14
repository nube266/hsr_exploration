#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_start_state import hsr_SetStartTimeState
from hsr_flexbe_states.hsr_check_elapsed_time_state import hsr_CheckElapsedTimeState
from flexbe_states.wait_state import WaitState
from hsr_flexbe_states.hsr_wait_dyn_state import WaitDynState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Aug 13 2019
@author: ShigemichiMatsuzaki
'''
class TestnewtimerSM(Behavior):
	'''
	A behavior to test 'hsr_CheckElapsedGlobalTime' state
	'''


	def __init__(self):
		super(TestnewtimerSM, self).__init__()
		self.name = 'Test new timer'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:478, x:130 y:478
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:62 y:60
			OperatableStateMachine.add('Start',
										hsr_SetStartTimeState(use_param=True),
										transitions={'succeeded': 'CheckElapsedTime'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'start_time': 'start_time'})

			# x:112 y:183
			OperatableStateMachine.add('CheckElapsedTime',
										hsr_CheckElapsedTimeState(time_limit=1.0, margin=0.33, use_param=True),
										transitions={'time_remains': 'Wait', 'time_up': 'wait'},
										autonomy={'time_remains': Autonomy.Off, 'time_up': Autonomy.Off},
										remapping={'start_time': 'start_time', 'offset': 'offset'})

			# x:363 y:236
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=5.0),
										transitions={'done': 'CheckElapsedTime'},
										autonomy={'done': Autonomy.Off})

			# x:154 y:326
			OperatableStateMachine.add('wait',
										WaitDynState(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'wait_time': 'offset'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
