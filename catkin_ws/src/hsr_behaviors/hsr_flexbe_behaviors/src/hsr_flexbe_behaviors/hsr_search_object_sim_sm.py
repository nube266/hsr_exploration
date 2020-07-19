#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_reset_octomap_state import hsr_ResetOctomapState
from hsr_flexbe_states.hsr_update_octomap_state import hsr_UpdateOctomapState
from hsr_flexbe_states.hsr_generating_candidates_state import hsr_GeneratingCandidatesState
from hsr_flexbe_behaviors.hsr_object_search_pose_sm import hsr_object_search_poseSM
from hsr_flexbe_states.hsr_object_detection_state import hsr_ObjectDetectionState
from hsr_flexbe_states.hsr_viewpoint_evaluator import hsr_ViewpointEvaluatorState
from hsr_flexbe_states.hsr_move_to_viewpoint_state import hsr_MoveToViewpointState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue May 19 2020
@author: Yusuke Miake
'''
class HSRSearchObjectSimSM(Behavior):
	'''
	Simulation of object search
	'''


	def __init__(self):
		super(HSRSearchObjectSimSM, self).__init__()
		self.name = 'HSR Search Object Sim'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(hsr_object_search_poseSM, 'ChangeOrientationForObjectDetection')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:915 y:298, x:132 y:271
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:136 y:27
			OperatableStateMachine.add('ResetOctomap',
										hsr_ResetOctomapState(srv_name="/octomap_server/reset"),
										transitions={'succeeded': 'UpdateOctomap'},
										autonomy={'succeeded': Autonomy.Off})

			# x:404 y:26
			OperatableStateMachine.add('UpdateOctomap',
										hsr_UpdateOctomapState(srv_name="/octomap_publisher/update_octomap", timeout=10.0),
										transitions={'succeeded': 'GenerateCandidates', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:658 y:26
			OperatableStateMachine.add('GenerateCandidates',
										hsr_GeneratingCandidatesState(srv_name="/viewpoint_planner_3d/generating_candidates", timeout=10.0),
										transitions={'succeeded': 'ChangeOrientationForObjectDetection', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:626 y:149
			OperatableStateMachine.add('ChangeOrientationForObjectDetection',
										self.use_behavior(hsr_object_search_poseSM, 'ChangeOrientationForObjectDetection'),
										transitions={'finished': 'DetectObject', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:677 y:286
			OperatableStateMachine.add('DetectObject',
										hsr_ObjectDetectionState(target_object_1="keyboard", target_object_2="", target_object_3="", timeout=2, bounding_box_topic="/darknet_ros/bounding_boxes"),
										transitions={'found': 'finished', 'not_found': 'EvaluateViewpoint'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off})

			# x:670 y:404
			OperatableStateMachine.add('EvaluateViewpoint',
										hsr_ViewpointEvaluatorState(srv_name="/viewpoint_planner_3d/get_next_viewpoint"),
										transitions={'succeeded': 'MoveToNextViewpoint', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:405 y:266
			OperatableStateMachine.add('MoveToNextViewpoint',
										hsr_MoveToViewpointState(),
										transitions={'succeeded': 'UpdateOctomap', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
