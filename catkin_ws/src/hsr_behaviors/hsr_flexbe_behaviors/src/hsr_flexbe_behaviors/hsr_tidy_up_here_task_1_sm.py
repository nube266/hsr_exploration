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
from hsr_flexbe_behaviors.hsr_tidy_up_here_open_drawers_sm import HSRTidyUpHereOpenDrawersSM
from hsr_flexbe_behaviors.hsr_tidy_up_here_task_1_tidy_up_sub_sm import HSRTidyUpHereTask1TidyUpSubSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed July 20 2019
@author: ShigemichiMatsuzaki
'''
class HSRTidyUpHereTask1SM(Behavior):
	'''
	Behavior of Tidy Up Here Task 1 (Tidying up clattered objects on the floor, the coffee table, and the side table)
	'''


	def __init__(self):
		super(HSRTidyUpHereTask1SM, self).__init__()
		self.name = 'HSR Tidy Up Here Task 1'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(HSRTidyUpHereOpenDrawersSM, 'HSR Tidy Up Here Open Drawers')
		self.add_behavior(HSRTidyUpHereTask1TidyUpSubSM, 'HSR Tidy Up Here Task 1 Tidy Up Sub')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:698 y:276, x:1055 y:202
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['start_time'])
		_state_machine.userdata.start_time = 0.0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:182 y:26
			OperatableStateMachine.add('Speak',
										hsr_SpeakState(sentence='I am going to tidy up here', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'HSR Tidy Up Here Open Drawers'},
										autonomy={'done': Autonomy.Off})

			# x:326 y:22
			OperatableStateMachine.add('HSR Tidy Up Here Open Drawers',
										self.use_behavior(HSRTidyUpHereOpenDrawersSM, 'HSR Tidy Up Here Open Drawers'),
										transitions={'finished': 'HSR Tidy Up Here Task 1 Tidy Up Sub', 'failed': 'HSR Tidy Up Here Task 1 Tidy Up Sub'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:575 y:101
			OperatableStateMachine.add('HSR Tidy Up Here Task 1 Tidy Up Sub',
										self.use_behavior(HSRTidyUpHereTask1TidyUpSubSM, 'HSR Tidy Up Here Task 1 Tidy Up Sub'),
										transitions={'finished': 'finished', 'failed': 'finished'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'start_time': 'start_time'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
