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
from hsr_flexbe_behaviors.hsr_tidy_up_here_task_2b_sm import HSRTidyUpHereTask2bSM
from hsr_flexbe_behaviors.hsr_tidy_up_here_task_1_sm import HSRTidyUpHereTask1SM
from hsr_flexbe_behaviors.hsr_tidy_up_here_task_2a_sm import HSRTidyUpHereTask2aSM
from hsr_flexbe_states.hsr_start_state import hsr_SetStartTimeState
from hsr_flexbe_behaviors.hsr_tidy_up_here_listen_new_sm import HSRTidyUpHereListenNewSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jul 20 2019
@author: ShigemichiMatsuzaki
'''
class HSRTidyUpHereSM(Behavior):
	'''
	Whole behavior of Tidy Up Here in Robocup Japan 2019 / WRS 2020
	'''


	def __init__(self):
		super(HSRTidyUpHereSM, self).__init__()
		self.name = 'HSR Tidy Up Here'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(HSRTidyUpHereTask2bSM, 'HSR Tidy Up Here Task 2b')
		self.add_behavior(HSRTidyUpHereTask1SM, 'HSR Tidy Up Here Task 1')
		self.add_behavior(HSRTidyUpHereTask2aSM, 'HSR Tidy Up Here Task 2a')
		self.add_behavior(HSRTidyUpHereListenNewSM, 'HSR Tidy Up Here Listen New')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:478, x:619 y:208
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.target_name = 'oolongtea'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:31 y:42
			OperatableStateMachine.add('StartButton',
										hsr_WaitWristPressedState(),
										transitions={'succeeded': 'HSR Tidy Up Here Listen New'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'start_time': 'start_time'})

			# x:205 y:530
			OperatableStateMachine.add('HSR Tidy Up Here Task 2b',
										self.use_behavior(HSRTidyUpHereTask2bSM, 'HSR Tidy Up Here Task 2b'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'target_name': 'object_name'})

			# x:296 y:222
			OperatableStateMachine.add('HSR Tidy Up Here Task 1',
										self.use_behavior(HSRTidyUpHereTask1SM, 'HSR Tidy Up Here Task 1'),
										transitions={'finished': 'HSR Tidy Up Here Task 2a', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'start_time': 'start_time'})

			# x:250 y:388
			OperatableStateMachine.add('HSR Tidy Up Here Task 2a',
										self.use_behavior(HSRTidyUpHereTask2aSM, 'HSR Tidy Up Here Task 2a'),
										transitions={'finished': 'HSR Tidy Up Here Task 2b', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:167 y:153
			OperatableStateMachine.add('SetStartTime',
										hsr_SetStartTimeState(),
										transitions={'succeeded': 'HSR Tidy Up Here Task 1'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'start_time': 'start_time'})

			# x:291 y:19
			OperatableStateMachine.add('HSR Tidy Up Here Listen New',
										self.use_behavior(HSRTidyUpHereListenNewSM, 'HSR Tidy Up Here Listen New'),
										transitions={'finished': 'StartButton2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'object_name': 'object_name'})

			# x:324 y:105
			OperatableStateMachine.add('StartButton2',
										hsr_WaitWristPressedState(),
										transitions={'succeeded': 'SetStartTime'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'start_time': 'start_time'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
