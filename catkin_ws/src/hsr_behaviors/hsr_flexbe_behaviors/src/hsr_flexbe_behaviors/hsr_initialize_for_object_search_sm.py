#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_start_timer_state import hsr_StartTimerState
from hsr_flexbe_states.hsr_reset_octomap_state import hsr_ResetOctomapState
from hsr_flexbe_states.hsr_update_octomap_state import hsr_UpdateOctomapState
from hsr_flexbe_states.hsr_move_to_neutral_state import hsr_MoveToNeutralState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jul 23 2020
@author: Yusuke Miake
'''
class hsr_initialize_for_object_searchSM(Behavior):
	'''
	Initialize for object search
	'''


	def __init__(self):
		super(hsr_initialize_for_object_searchSM, self).__init__()
		self.name = 'hsr_initialize_for_object_search'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:843 y:50, x:168 y:271
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:75 y:103
			OperatableStateMachine.add('MoveToNeutral',
										hsr_MoveToNeutralState(open_hand=False),
										transitions={'succeeded': 'Start Timer', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:328 y:41
			OperatableStateMachine.add('Reset Octomap',
										hsr_ResetOctomapState(srv_name="/octomap_server/reset"),
										transitions={'succeeded': 'Update Octomap'},
										autonomy={'succeeded': Autonomy.Off})

			# x:577 y:36
			OperatableStateMachine.add('Update Octomap',
										hsr_UpdateOctomapState(srv_name="/octomap_publisher/update_octomap", timeout=10.0),
										transitions={'succeeded': 'finished'},
										autonomy={'succeeded': Autonomy.Off})

			# x:348 y:153
			OperatableStateMachine.add('Start Timer',
										hsr_StartTimerState(param_name="/viewpoint_planner_viewer/start_total_search_timer"),
										transitions={'succeeded': 'Reset Octomap'},
										autonomy={'succeeded': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
