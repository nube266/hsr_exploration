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
from hsr_flexbe_behaviors.hsr_tidy_up_here_task_1_tidy_up_sm import HSRTidyUpHereTask1TidyUpSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jul 30 2019
@author: ShigemichiMatsuzaki
'''
class HSRTidyUpTestSM(Behavior):
	'''
	Behavior to test only tidy up process in Tidy Up Here Task 1
	'''


	def __init__(self):
		super(HSRTidyUpTestSM, self).__init__()
		self.name = 'HSR Tidy Up Test'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(HSRTidyUpHereTask1TidyUpSM, 'HSR Tidy Up Here Task 1 Tidy Up')

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
			# x:30 y:40
			OperatableStateMachine.add('Start',
										hsr_SetStartTimeState(),
										transitions={'succeeded': 'HSR Tidy Up Here Task 1 Tidy Up'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'start_time': 'start_time'})

			# x:157 y:151
			OperatableStateMachine.add('HSR Tidy Up Here Task 1 Tidy Up',
										self.use_behavior(HSRTidyUpHereTask1TidyUpSM, 'HSR Tidy Up Here Task 1 Tidy Up'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'start_time': 'start_time'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
