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
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBaseState
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
		# x:742 y:75, x:916 y:245
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:142 y:63
			OperatableStateMachine.add('SetGoal',
										hsr_SetBasePoseByTfNameState(tf_name='shelf', service_name='/pose_server/getPose'),
										transitions={'completed': 'Task2aState'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:441 y:64
			OperatableStateMachine.add('Task2aState',
										hsr_Task2aState(move_srv_name="/avoidance_move_server/move", reachable_area_upper_left_x=2.1, reachable_area_upper_left_y=-10000, reachable_area_bottom_right_x=10000, reachable_area_bottom_right_y=-1.5, obstacle_area_size=0.55),
										transitions={'succeeded': 'finished', 'failed': 'FailedSetGoal'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:409 y:233
			OperatableStateMachine.add('FailedSetGoal',
										hsr_SetBasePoseByTfNameState(tf_name='shelf', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveGoal'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:694 y:236
			OperatableStateMachine.add('MoveGoal',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'finished', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
