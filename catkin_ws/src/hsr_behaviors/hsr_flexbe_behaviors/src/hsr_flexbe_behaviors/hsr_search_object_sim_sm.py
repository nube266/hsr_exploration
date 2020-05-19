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
from hsr_flexbe_states.hsr_object_detection_state import hsr_ObjectDetectionState
from hsr_flexbe_states.hsr_viewpoint_evaluator import hsr_ViewpointEvaluatorState
from hsr_flexbe_states.hsr_move_state_for_gazebo import hsr_MoveStateForGazebo
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

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:829 y:170, x:228 y:273
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:79 y:34
			OperatableStateMachine.add('ResetOctomap',
										hsr_ResetOctomapState(srv_name="/octomap_server/reset"),
										transitions={'succeeded': 'GenerateCandidates'},
										autonomy={'succeeded': Autonomy.Off})

			# x:590 y:27
			OperatableStateMachine.add('UpdateOctomap',
										hsr_UpdateOctomapState(srv_name="/octomap_publisher/update_octomap", timeout=10.0),
										transitions={'succeeded': 'ObjectDetection', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:321 y:30
			OperatableStateMachine.add('GenerateCandidates',
										hsr_GeneratingCandidatesState(),
										transitions={'succeeded': 'UpdateOctomap', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:592 y:160
			OperatableStateMachine.add('ObjectDetection',
										hsr_ObjectDetectionState(),
										transitions={'found': 'finished', 'not_found': 'ViewpointEvaluator'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off})

			# x:590 y:292
			OperatableStateMachine.add('ViewpointEvaluator',
										hsr_ViewpointEvaluatorState(),
										transitions={'succeeded': 'Move', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:328 y:359
			OperatableStateMachine.add('Move',
										hsr_MoveStateForGazebo(),
										transitions={'succeeded': 'UpdateOctomap', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
