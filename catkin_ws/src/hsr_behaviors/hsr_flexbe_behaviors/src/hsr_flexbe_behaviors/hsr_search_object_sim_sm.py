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
		self.add_behavior(hsr_object_search_poseSM, 'hsr_object_search_pose')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:907 y:266, x:235 y:401
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:117 y:28
			OperatableStateMachine.add('ResetOctomap',
										hsr_ResetOctomapState(srv_name="/octomap_server/reset"),
										transitions={'succeeded': 'UpdateOctomap'},
										autonomy={'succeeded': Autonomy.Off})

			# x:404 y:26
			OperatableStateMachine.add('UpdateOctomap',
										hsr_UpdateOctomapState(srv_name="/octomap_publisher/update_octomap", timeout=10.0),
										transitions={'succeeded': 'GeneratingCandidates', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:651 y:24
			OperatableStateMachine.add('GeneratingCandidates',
										hsr_GeneratingCandidatesState(srv_name="/viewpoint_planner_3d/generating_candidates", timeout=10.0),
										transitions={'succeeded': 'hsr_object_search_pose', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:660 y:144
			OperatableStateMachine.add('hsr_object_search_pose',
										self.use_behavior(hsr_object_search_poseSM, 'hsr_object_search_pose'),
										transitions={'finished': 'ObjectDetection', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:682 y:285
			OperatableStateMachine.add('ObjectDetection',
										hsr_ObjectDetectionState(target_object_1="apple", target_object_2="orange", target_object_3="", timeout=1, bounding_box_topic="/darknet_ros/bounding_boxes"),
										transitions={'found': 'finished', 'not_found': 'UpdateOctomap'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
