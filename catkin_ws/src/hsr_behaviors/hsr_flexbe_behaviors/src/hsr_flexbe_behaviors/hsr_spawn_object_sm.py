#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_get_random_target_pose_state import hsr_GetRandomTargetPoseState
from hsr_flexbe_states.hsr_spawn_model_to_input import hsr_SpawnModelToInputState
from hsr_flexbe_states.hsr_delete_model_state import hsr_DeleteModelState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Jul 26 2020
@author: Yusuke Miake
'''
class hsr_spawn_objectSM(Behavior):
	'''
	The behavior that causes the object to spawn
	'''


	def __init__(self):
		super(hsr_spawn_objectSM, self).__init__()
		self.name = 'hsr_spawn_object'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:725 y:168, x:668 y:249
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:128 y:34
			OperatableStateMachine.add('DeleteModel',
										hsr_DeleteModelState(model_name="cup_blue", srv_name="/gazebo/delete_model"),
										transitions={'succeeded': 'GetRandomPose'},
										autonomy={'succeeded': Autonomy.Off})

			# x:406 y:158
			OperatableStateMachine.add('SpawnModel',
										hsr_SpawnModelToInputState(model_path="/root/HSR/catkin_ws/src/hsr_object_search_world/models/cup_blue/model.sdf", model_name="cup_blue"),
										transitions={'succeeded': 'finished'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:116 y:157
			OperatableStateMachine.add('GetRandomPose',
										hsr_GetRandomTargetPoseState(),
										transitions={'succeeded': 'SpawnModel'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
