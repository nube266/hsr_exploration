#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_delete_model_state import hsr_DeleteModelState
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBaseState
from hsr_flexbe_states.hsr_restart_gmapping_state import hsr_RestartGmappingState
from hsr_flexbe_states.hsr_reset_octomap_state import hsr_ResetOctomapState
from flexbe_states.wait_state import WaitState
from hsr_flexbe_states.hsr_get_random_robot_pose import hsr_GetRandomRobotPose
from hsr_flexbe_states.hsr_spawn_model_state import hsr_SpawnModelState
from hsr_flexbe_states.hsr_reset_viewer_state import hsr_ResetViewerState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jul 24 2020
@author: YusukeMiake
'''
class hsr_setup_for_experimentSM(Behavior):
	'''
	Setup for Experiment
	'''


	def __init__(self):
		super(hsr_setup_for_experimentSM, self).__init__()
		self.name = 'hsr_setup_for_experiment'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:831 y:564, x:523 y:232
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:58 y:28
			OperatableStateMachine.add('DeleteObject',
										hsr_DeleteModelState(model_name="cup_blue", srv_name="/gazebo/delete_model"),
										transitions={'succeeded': 'GetRandomTargetPose'},
										autonomy={'succeeded': Autonomy.Off})

			# x:282 y:225
			OperatableStateMachine.add('MoveToInitialPose',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'ResetOctomap', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:510 y:317
			OperatableStateMachine.add('ResetMap',
										hsr_RestartGmappingState(),
										transitions={'succeeded': 'ResetViewer'},
										autonomy={'succeeded': Autonomy.Off})

			# x:275 y:318
			OperatableStateMachine.add('ResetOctomap',
										hsr_ResetOctomapState(srv_name="/octomap_server/reset"),
										transitions={'succeeded': 'ResetMap'},
										autonomy={'succeeded': Autonomy.Off})

			# x:793 y:438
			OperatableStateMachine.add('WaitSetup',
										WaitState(wait_time=15),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:271 y:116
			OperatableStateMachine.add('GetRandomRobotPose',
										hsr_GetRandomRobotPose(),
										transitions={'succeeded': 'MoveToInitialPose'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:48 y:121
			OperatableStateMachine.add('GetRandomTargetPose',
										hsr_GetRandomRobotPose(),
										transitions={'succeeded': 'SpawnTarget'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:56 y:234
			OperatableStateMachine.add('SpawnTarget',
										hsr_SpawnModelState(model_path="/root/HSR/catkin_ws/src/hsr_object_search_world/models/book_1/model.sdf", model_name="book_1", model_format="sdf", x=0.0, y=0.0, z=0.0),
										transitions={'succeeded': 'GetRandomRobotPose'},
										autonomy={'succeeded': Autonomy.Off})

			# x:763 y:322
			OperatableStateMachine.add('ResetViewer',
										hsr_ResetViewerState(),
										transitions={'succeeded': 'WaitSetup'},
										autonomy={'succeeded': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
