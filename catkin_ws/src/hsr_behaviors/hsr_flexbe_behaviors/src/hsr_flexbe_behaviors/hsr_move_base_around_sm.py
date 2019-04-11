#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_set_base_pose_state import hsr_SetBasePoseState
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBaseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Apr 09 2019
@author: Lotfi El Hafi
'''
class HSRMoveBaseAroundSM(Behavior):
	'''
	A simple behavior to move the HSR around predifined waypoints. The main purpose of this behavior is to quickly test the development environment and the tools integration.
	'''


	def __init__(self):
		super(HSRMoveBaseAroundSM, self).__init__()
		self.name = 'HSR Move Base Around'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:118 y:217, x:737 y:225
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		# [/MANUAL_CREATE]


		with _state_machine:
			# x:231 y:89
			OperatableStateMachine.add('hsr_SetBasePoseState1',
										hsr_SetBasePoseState(pose_position_x=0.5, pose_position_y=0.0, pose_orientation_z=0.0, pose_orientation_w=1.0),
										transitions={'completed': 'hsr_MoveBaseState1'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose1'})

			# x:536 y:88
			OperatableStateMachine.add('hsr_MoveBaseState1',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'hsr_SetBasePoseState2', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose1'})

			# x:843 y:77
			OperatableStateMachine.add('hsr_SetBasePoseState2',
										hsr_SetBasePoseState(pose_position_x=0.5, pose_position_y=0.5, pose_orientation_z=0.0, pose_orientation_w=1.0),
										transitions={'completed': 'hsr_MoveBaseState2'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose2'})

			# x:1211 y:78
			OperatableStateMachine.add('hsr_MoveBaseState2',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'hsr_SetBasePoseState3', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose2'})

			# x:871 y:334
			OperatableStateMachine.add('hsr_MoveBaseState3',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'hsr_SetBasePoseState4', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose3'})

			# x:232 y:334
			OperatableStateMachine.add('hsr_MoveBaseState4',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'finished', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose4'})

			# x:1194 y:340
			OperatableStateMachine.add('hsr_SetBasePoseState3',
										hsr_SetBasePoseState(pose_position_x=0.0, pose_position_y=0.5, pose_orientation_z=0.0, pose_orientation_w=1.0),
										transitions={'completed': 'hsr_MoveBaseState3'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose3'})

			# x:525 y:335
			OperatableStateMachine.add('hsr_SetBasePoseState4',
										hsr_SetBasePoseState(pose_position_x=0.0, pose_position_y=0.0, pose_orientation_z=0.0, pose_orientation_w=1.0),
										transitions={'completed': 'hsr_MoveBaseState4'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose4'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	# [/MANUAL_FUNC]
