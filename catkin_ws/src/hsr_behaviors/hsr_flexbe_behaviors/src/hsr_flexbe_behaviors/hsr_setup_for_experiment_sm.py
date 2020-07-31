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
from hsr_flexbe_states.hsr_reset_viewer_state import hsr_ResetViewerState
from hsr_flexbe_states.hsr_spawn_model_to_input import hsr_SpawnModelToInputState
from hsr_flexbe_states.hsr_set_pose_state import hsr_SetBasePoseState
from hsr_flexbe_states.hsr_get_object_pose_state import hsr_GetObjectPoseState
from hsr_flexbe_states.hsr_object_detection_state import hsr_ObjectDetectionState
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
		# x:805 y:466, x:523 y:232
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:58 y:28
			OperatableStateMachine.add('DeleteObject',
										hsr_DeleteModelState(model_name="sports_ball", srv_name="/gazebo/delete_model"),
										transitions={'succeeded': 'GetTargetPose'},
										autonomy={'succeeded': Autonomy.Off})

			# x:298 y:223
			OperatableStateMachine.add('MoveToInitialPose',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'ResetOctomap', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:281 y:437
			OperatableStateMachine.add('ResetMap',
										hsr_RestartGmappingState(),
										transitions={'succeeded': 'ResetViewer'},
										autonomy={'succeeded': Autonomy.Off})

			# x:292 y:319
			OperatableStateMachine.add('ResetOctomap',
										hsr_ResetOctomapState(srv_name="/octomap_server/reset"),
										transitions={'succeeded': 'ResetMap'},
										autonomy={'succeeded': Autonomy.Off})

			# x:778 y:305
			OperatableStateMachine.add('WaitSetup',
										WaitState(wait_time=5),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:523 y:436
			OperatableStateMachine.add('ResetViewer',
										hsr_ResetViewerState(),
										transitions={'succeeded': 'ResetObjectDetector'},
										autonomy={'succeeded': Autonomy.Off})

			# x:44 y:243
			OperatableStateMachine.add('SpawnTarget',
										hsr_SpawnModelToInputState(model_path="/root/HSR/catkin_ws/src/hsr_object_search_world/models/sports_ball/model.sdf", model_name="sports_ball"),
										transitions={'succeeded': 'SetInitialPose'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:292 y:104
			OperatableStateMachine.add('SetInitialPose',
										hsr_SetBasePoseState(x=0.0, y=0.0, yaw=0.0),
										transitions={'succeeded': 'MoveToInitialPose'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:52 y:125
			OperatableStateMachine.add('GetTargetPose',
										hsr_GetObjectPoseState(pose_list_path="/root/HSR/catkin_ws/src/hsr_behaviors/hsr_flexbe_states/config/target_object_area_01.json", save_data_path="/root/HSR/catkin_ws/src/hsr_behaviors/hsr_flexbe_states/src/hsr_flexbe_states/data/log.csv"),
										transitions={'succeeded': 'SpawnTarget'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:519 y:308
			OperatableStateMachine.add('ResetObjectDetector',
										hsr_ObjectDetectionState(target_object_1="", target_object_2="", target_object_3="", timeout=2.0, bounding_box_topic="/darknet_ros/bounding_boxes"),
										transitions={'found': 'WaitSetup', 'not_found': 'WaitSetup'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
