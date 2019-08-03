#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_set_base_pose_by_tf_name_state import hsr_SetBasePoseByTfNameState
from hsr_flexbe_states.hsr_task2a_state import hsr_Task2aState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed July 3 2019
@author: YusukeMiake
'''
class HSRTask2aSM(Behavior):
	'''
	Source code for task 2a.
	'''


	def __init__(self):
		super(HSRTask2aSM, self).__init__()
		self.name = 'HSR Task2a'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:675 y:71, x:678 y:260
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:120 y:65
			OperatableStateMachine.add('SetGoalPose',
										hsr_SetBasePoseByTfNameState(tf_name='shelf', service_name='/pose_server/getPose'),
										transitions={'completed': 'Task2aMove'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:429 y:69
			OperatableStateMachine.add('Task2aMove',
										hsr_Task2aState(move_srv_name="/avoidance_move_server/move"),
										transitions={'succeeded': 'finished', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
