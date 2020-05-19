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
from hsr_flexbe_states.hsr_get_next_viewpoint import hsr_GetNextViewpoint
from hsr_flexbe_states.hsr_move_state_for_gazebo import hsr_MoveStateForGazebo
from hsr_flexbe_states.hsr_update_octomap_state import hsr_UpdateOctomapState
from hsr_flexbe_states.hsr_object_detection_state import hsr_ObjectDetectionState
from flexbe_states.wait_state import WaitState
from hsr_flexbe_states.hsr_move_to_go_state import hsr_MoveToGoState
from hsr_flexbe_states.hsr_head_move_state import hsr_HeadMoveState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Nov 20 2019
@author: YusukeMiake
'''
class search_table_2019SM(Behavior):
	'''
	The Behavior used in 2019 study (B4)
	'''


	def __init__(self):
		super(search_table_2019SM, self).__init__()
		self.name = 'search_table_2019'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:742 y:501, x:359 y:44
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:60 y:29
			OperatableStateMachine.add('ResetOctomap',
										hsr_ResetOctomapState(srv_name="/octomap_server/reset"),
										transitions={'succeeded': 'InitializationOctomap'},
										autonomy={'succeeded': Autonomy.Off})

			# x:695 y:258
			OperatableStateMachine.add('GetNextViewpoint',
										hsr_GetNextViewpoint(srv_name="/viewpoint_planner/get_next_viewpoint"),
										transitions={'succeeded': 'NeutralForMove', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:684 y:33
			OperatableStateMachine.add('MoveNextViewpoint',
										hsr_MoveStateForGazebo(),
										transitions={'succeeded': 'MoveHeadTilt', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:356 y:386
			OperatableStateMachine.add('UpdateOctomap',
										hsr_UpdateOctomapState(srv_name="/octomap_publisher/update_octomap", timeout=10.0),
										transitions={'succeeded': 'ObjectDetection', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:683 y:381
			OperatableStateMachine.add('ObjectDetection',
										hsr_ObjectDetectionState(),
										transitions={'found': 'finished', 'not_found': 'GetNextViewpoint'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off})

			# x:75 y:268
			OperatableStateMachine.add('WaitSetup',
										WaitState(wait_time=5),
										transitions={'done': 'MoveHeadTilt'},
										autonomy={'done': Autonomy.Off})

			# x:59 y:146
			OperatableStateMachine.add('InitializationOctomap',
										hsr_UpdateOctomapState(srv_name="/octomap_publisher/update_octomap", timeout=10.0),
										transitions={'succeeded': 'WaitSetup', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:698 y:137
			OperatableStateMachine.add('NeutralForMove',
										hsr_MoveToGoState(),
										transitions={'succeeded': 'MoveNextViewpoint', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:68 y:386
			OperatableStateMachine.add('MoveHeadTilt',
										hsr_HeadMoveState(head_position=-0.5),
										transitions={'succeeded': 'UpdateOctomap'},
										autonomy={'succeeded': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
