#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_behaviors.hsr_initialize_for_object_search_sm import hsr_initialize_for_object_searchSM
from hsr_flexbe_states.hsr_update_octomap_state import hsr_UpdateOctomapState
from hsr_flexbe_states.hsr_object_detection_state import hsr_ObjectDetectionState
from hsr_flexbe_states.hsr_stop_timer_state import hsr_StopTimerState
from hsr_flexbe_states.hsr_save_viewer_data_state import hsr_SaveViewerDataState
from hsr_flexbe_states.hsr_generating_candidates_state import hsr_GeneratingCandidatesState
from hsr_flexbe_states.hsr_viewpoint_evaluator import hsr_ViewpointEvaluatorState
from hsr_flexbe_states.hsr_move_to_viewpoint_state import hsr_MoveToViewpointState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 29 2020
@author: Yusuke Miake
'''
class hsr_object_search_simulationSM(Behavior):
	'''
	Simulation of object search
	'''


	def __init__(self):
		super(hsr_object_search_simulationSM, self).__init__()
		self.name = 'hsr_object_search_simulation'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(hsr_initialize_for_object_searchSM, 'hsr_initialize_for_object_search')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1107 y:337, x:168 y:252
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:80 y:72
			OperatableStateMachine.add('hsr_initialize_for_object_search',
										self.use_behavior(hsr_initialize_for_object_searchSM, 'hsr_initialize_for_object_search'),
										transitions={'finished': 'Updateoctomap', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:382 y:48
			OperatableStateMachine.add('Updateoctomap',
										hsr_UpdateOctomapState(srv_name="/octomap_publisher/update_octomap", timeout=10.0),
										transitions={'succeeded': 'DetectObject'},
										autonomy={'succeeded': Autonomy.Off})

			# x:601 y:56
			OperatableStateMachine.add('DetectObject',
										hsr_ObjectDetectionState(target_object_1="sports ball", target_object_2="", target_object_3="", timeout=1.0, bounding_box_topic="/darknet_ros/bounding_boxes"),
										transitions={'found': 'StopTimer', 'not_found': 'GenerateCandidates'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off})

			# x:851 y:200
			OperatableStateMachine.add('StopTimer',
										hsr_StopTimerState(param_name="/viewpoint_planner_viewer/start_total_search_timer"),
										transitions={'succeeded': 'SaveData'},
										autonomy={'succeeded': Autonomy.Off})

			# x:1051 y:198
			OperatableStateMachine.add('SaveData',
										hsr_SaveViewerDataState(save_path="/root/HSR/catkin_ws/src/hsr_behaviors/hsr_flexbe_states/src/hsr_flexbe_states/data/log.csv", success=True),
										transitions={'succeeded': 'finished'},
										autonomy={'succeeded': Autonomy.Off})

			# x:587 y:200
			OperatableStateMachine.add('GenerateCandidates',
										hsr_GeneratingCandidatesState(srv_name="/viewpoint_planner_3d/generating_candidates", timeout=10.0),
										transitions={'succeeded': 'EvaluateViewpoint'},
										autonomy={'succeeded': Autonomy.Off})

			# x:591 y:303
			OperatableStateMachine.add('EvaluateViewpoint',
										hsr_ViewpointEvaluatorState(srv_name="/viewpoint_planner_3d/get_next_viewpoint"),
										transitions={'succeeded': 'MoveToNextViewpoint'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:596 y:457
			OperatableStateMachine.add('SaveFailed',
										hsr_SaveViewerDataState(save_path="/root/HSR/catkin_ws/src/hsr_behaviors/hsr_flexbe_states/src/hsr_flexbe_states/data/log.csv", success=False),
										transitions={'succeeded': 'finished'},
										autonomy={'succeeded': Autonomy.Off})

			# x:372 y:299
			OperatableStateMachine.add('MoveToNextViewpoint',
										hsr_MoveToViewpointState(),
										transitions={'succeeded': 'Updateoctomap', 'failed': 'SaveFailed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
