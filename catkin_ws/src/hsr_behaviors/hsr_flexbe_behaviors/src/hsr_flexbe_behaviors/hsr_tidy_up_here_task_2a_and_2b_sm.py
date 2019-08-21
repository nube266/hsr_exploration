#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_behaviors.hsr_tidy_up_here_task_2a_sm import HSRTidyUpHereTask2aSM
from hsr_flexbe_behaviors.hsr_tidy_up_here_task_2b_sm import HSRTidyUpHereTask2bSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jul 20 2019
@author: ShigemichiMatsuzaki
'''
class HSRTidyUpHereTask2aand2bSM(Behavior):
	'''
	Whole behavior of Tidy Up Here in Robocup Japan 2019 / WRS 2020
	'''


	def __init__(self):
		super(HSRTidyUpHereTask2aand2bSM, self).__init__()
		self.name = 'HSR Tidy Up Here Task 2a and 2b'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(HSRTidyUpHereTask2aSM, 'HSR Tidy Up Here Task 2a')
		self.add_behavior(HSRTidyUpHereTask2bSM, 'HSR Tidy Up Here Task 2b')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:478, x:619 y:208
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['object_name'])
		_state_machine.userdata.object_name = 'oolongtea'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:162 y:185
			OperatableStateMachine.add('HSR Tidy Up Here Task 2a',
										self.use_behavior(HSRTidyUpHereTask2aSM, 'HSR Tidy Up Here Task 2a'),
										transitions={'finished': 'HSR Tidy Up Here Task 2b', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:205 y:530
			OperatableStateMachine.add('HSR Tidy Up Here Task 2b',
										self.use_behavior(HSRTidyUpHereTask2bSM, 'HSR Tidy Up Here Task 2b'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'target_name': 'object_name'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
