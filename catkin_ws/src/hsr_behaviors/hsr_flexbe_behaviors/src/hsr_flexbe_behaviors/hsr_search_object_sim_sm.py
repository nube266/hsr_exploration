#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_behaviors.initializeforobjectsearch_sm import InitializeForObjectSearchSM
from hsr_flexbe_behaviors.hsr_object_search_pose_sm import hsr_object_search_poseSM
from hsr_flexbe_states.hsr_object_detection_state import hsr_ObjectDetectionState
from hsr_flexbe_states.hsr_viewpoint_evaluator import hsr_ViewpointEvaluatorState
from hsr_flexbe_states.hsr_move_to_viewpoint_state import hsr_MoveToViewpointState
from hsr_flexbe_states.hsr_generating_candidates_state import hsr_GeneratingCandidatesState
from hsr_flexbe_states.hsr_update_octomap_state import hsr_UpdateOctomapState
from hsr_flexbe_states.hsr_stop_timer_state import hsr_StopTimerState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue May 19 2020
@author: Yusuke Miake
'''
class hsr_search_object_simSM(Behavior):
	'''
	Simulation of object search
	'''


	def __init__(self):
		super(hsr_search_object_simSM, self).__init__()
		self.name = 'hsr_search_object_sim'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(InitializeForObjectSearchSM, 'InitializeForObjectSearch')
		self.add_behavior(hsr_object_search_poseSM, 'ChangeOrientationForObjectDetection')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:653 y:460, x:163 y:192
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:102 y:22
			OperatableStateMachine.add('InitializeForObjectSearch',
										self.use_behavior(InitializeForObjectSearchSM, 'InitializeForObjectSearch'),
										transitions={'finished': 'Update octomap', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:365 y:174
			OperatableStateMachine.add('ChangeOrientationForObjectDetection',
										self.use_behavior(hsr_object_search_poseSM, 'ChangeOrientationForObjectDetection'),
										transitions={'finished': 'DetectObject', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:404 y:314
			OperatableStateMachine.add('DetectObject',
										hsr_ObjectDetectionState(target_object_1="keyboard", target_object_2="", target_object_3="", timeout=2, bounding_box_topic="/darknet_ros/bounding_boxes"),
										transitions={'found': 'TerminateTheSearching', 'not_found': 'GenerateCandidates'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off})

			# x:681 y:175
			OperatableStateMachine.add('EvaluateViewpoint',
										hsr_ViewpointEvaluatorState(srv_name="/viewpoint_planner_3d/get_next_viewpoint"),
										transitions={'succeeded': 'MoveToNextViewpoint'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:686 y:22
			OperatableStateMachine.add('MoveToNextViewpoint',
										hsr_MoveToViewpointState(),
										transitions={'succeeded': 'Update octomap', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:675 y:314
			OperatableStateMachine.add('GenerateCandidates',
										hsr_GeneratingCandidatesState(srv_name="/viewpoint_planner_3d/generating_candidates", timeout=10.0),
										transitions={'succeeded': 'EvaluateViewpoint'},
										autonomy={'succeeded': Autonomy.Off})

			# x:405 y:23
			OperatableStateMachine.add('Update octomap',
										hsr_UpdateOctomapState(srv_name="/octomap_publisher/update_octomap", timeout=10.0),
										transitions={'succeeded': 'ChangeOrientationForObjectDetection'},
										autonomy={'succeeded': Autonomy.Off})

			# x:406 y:442
			OperatableStateMachine.add('TerminateTheSearching',
										hsr_StopTimerState(param_name="/viewpoint_planner_viewer/start_total_search_timer"),
										transitions={'succeeded': 'finished'},
										autonomy={'succeeded': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
