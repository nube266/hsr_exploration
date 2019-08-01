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
from hsr_flexbe_states.hsr_move_to_neutral_state import hsr_MoveToNeutralState
from hsr_flexbe_behaviors.hsr_tidy_up_here_task_1_tidy_up_long_table_sm import HSRTidyUpHereTask1TidyUpLongTableSM
from hsr_flexbe_behaviors.hsr_tidy_up_here_task_1_tidy_up_tall_table_sm import HSRTidyUpHereTask1TidyUpTallTableSM
from hsr_flexbe_behaviors.hsr_tidy_up_here_task_1_tidy_up_floor_sm import HSRTidyUpHereTask1TidyUpFloorSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Aug 1 2019
@author: ShigemichiMatsuzaki
'''
class HSRTidyUpHereTask1TidyUpSubSM(Behavior):
	'''
	Test behavior
	'''


	def __init__(self):
		super(HSRTidyUpHereTask1TidyUpSubSM, self).__init__()
		self.name = 'HSR Tidy Up Here Task 1 Tidy Up Sub'

		# parameters of this behavior
		self.add_parameter('time_limit_task1', 15.0)

		# references to used behaviors
		self.add_behavior(HSRTidyUpHereTask1TidyUpLongTableSM, 'HSR Tidy Up Here Task 1 Tidy Up Long Table')
		self.add_behavior(HSRTidyUpHereTask1TidyUpTallTableSM, 'HSR Tidy Up Here Task 1 Tidy Up Tall Table')
		self.add_behavior(HSRTidyUpHereTask1TidyUpFloorSM, 'HSR Tidy Up Here Task 1 Tidy Up Floor')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1607 y:607, x:1376 y:171
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['start_time'])
		_state_machine.userdata.start_time = 5.0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:104 y:84
			OperatableStateMachine.add('HSR Tidy Up Here Task 1 Tidy Up Floor',
										self.use_behavior(HSRTidyUpHereTask1TidyUpFloorSM, 'HSR Tidy Up Here Task 1 Tidy Up Floor'),
										transitions={'finished': 'HSR Tidy Up Here Task 1 Tidy Up Long Table', 'failed': 'failed', 'time_up': 'SpeakTimeUp'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'time_up': Autonomy.Inherit},
										remapping={'start_time': 'start_time'})

			# x:1517 y:458
			OperatableStateMachine.add('MoveToNeutralTimeUp',
										hsr_MoveToNeutralState(open_hand=True),
										transitions={'succeeded': 'finished', 'failed': 'finished'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:183 y:240
			OperatableStateMachine.add('HSR Tidy Up Here Task 1 Tidy Up Long Table',
										self.use_behavior(HSRTidyUpHereTask1TidyUpLongTableSM, 'HSR Tidy Up Here Task 1 Tidy Up Long Table'),
										transitions={'finished': 'HSR Tidy Up Here Task 1 Tidy Up Tall Table', 'failed': 'failed', 'time_up': 'SpeakTimeUp'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'time_up': Autonomy.Inherit},
										remapping={'start_time': 'start_time'})

			# x:422 y:389
			OperatableStateMachine.add('HSR Tidy Up Here Task 1 Tidy Up Tall Table',
										self.use_behavior(HSRTidyUpHereTask1TidyUpTallTableSM, 'HSR Tidy Up Here Task 1 Tidy Up Tall Table'),
										transitions={'finished': 'finished', 'failed': 'failed', 'time_up': 'SpeakTimeUp'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'time_up': Autonomy.Inherit},
										remapping={'start_time': 'start_time'})

			# x:1526 y:222
			OperatableStateMachine.add('SpeakTimeUp',
										hsr_SpeakState(sentence='Time is up. Move on to the next task', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'MoveToNeutralTimeUp'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
