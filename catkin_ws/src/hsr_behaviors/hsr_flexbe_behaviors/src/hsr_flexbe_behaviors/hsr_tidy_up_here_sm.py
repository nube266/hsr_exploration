#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_behaviors.hsr_tidy_up_here_listen_sm import HSRTidyUpHereListenSM
from hsr_flexbe_behaviors.hsr_tidy_up_here_task_2b_sm import HSRTidyUpHereTask2bSM
from hsr_flexbe_behaviors.hsr_tidy_up_here_task_1_sm import HSRTidyUpHereTask1SM
from hsr_flexbe_behaviors.hsr_tidy_up_here_task_2a_sm import HSRTidyUpHereTask2aSM
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
		self.add_behavior(HSRTidyUpHereListenSM, 'HSR Tidy Up Here Listen')
		self.add_behavior(HSRTidyUpHereTask2bSM, 'HSR Tidy Up Here Task 2b')
		self.add_behavior(HSRTidyUpHereTask1SM, 'HSR Tidy Up Here Task 1')
		self.add_behavior(HSRTidyUpHereTask2aSM, 'HSR Tidy Up Here Task 2a')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:478, x:405 y:182
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.target_name = 'oolongtea'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:102 y:57
			OperatableStateMachine.add('HSR Tidy Up Here Listen',
										self.use_behavior(HSRTidyUpHereListenSM, 'HSR Tidy Up Here Listen'),
										transitions={'finished': 'HSR Tidy Up Here Task 1', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'object_name': 'object_name'})

			# x:248 y:424
			OperatableStateMachine.add('HSR Tidy Up Here Task 2b',
										self.use_behavior(HSRTidyUpHereTask2bSM, 'HSR Tidy Up Here Task 2b'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'target_name': 'object_name'})

			# x:187 y:182
			OperatableStateMachine.add('HSR Tidy Up Here Task 1',
										self.use_behavior(HSRTidyUpHereTask1SM, 'HSR Tidy Up Here Task 1'),
										transitions={'finished': 'HSR Tidy Up Here Task 2a', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:209 y:294
			OperatableStateMachine.add('HSR Tidy Up Here Task 2a',
										self.use_behavior(HSRTidyUpHereTask2aSM, 'HSR Tidy Up Here Task 2a'),
										transitions={'finished': 'HSR Tidy Up Here Task 2b', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
