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
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBaseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jun 08 2019
@author: ShigemichiMatsuzaki
'''
class HSRSetBasePoseByTfNameTestSM(Behavior):
	'''
	Test code of hsr_SetBasePoseByTfNameState.
Just move to several place.
Make sure to run roslaunch hsr_demonstration demo_sim_2019.launch.
	'''


	def __init__(self):
		super(HSRSetBasePoseByTfNameTestSM, self).__init__()
		self.name = 'HSR Set Base Pose By Tf Name Test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:310 y:543, x:298 y:383
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('SetPose1',
										hsr_SetBasePoseByTfNameState(tf_name='searching_point_0', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveBase1'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:102 y:126
			OperatableStateMachine.add('MoveBase1',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'SetPose2', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:262 y:172
			OperatableStateMachine.add('SetPose2',
										hsr_SetBasePoseByTfNameState(tf_name='rubbishbin', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveBase'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:407 y:251
			OperatableStateMachine.add('MoveBase',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'SetPose3', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:577 y:331
			OperatableStateMachine.add('SetPose3',
										hsr_SetBasePoseByTfNameState(tf_name='searching_point_0', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveBase3'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:678 y:416
			OperatableStateMachine.add('MoveBase3',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'SetPose4', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:702 y:502
			OperatableStateMachine.add('SetPose4',
										hsr_SetBasePoseByTfNameState(tf_name='rubbishbin', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveBase4'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:778 y:582
			OperatableStateMachine.add('MoveBase4',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'finished', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
