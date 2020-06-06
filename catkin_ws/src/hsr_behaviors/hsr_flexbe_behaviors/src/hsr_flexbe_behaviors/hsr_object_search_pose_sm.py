#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_move_to_neutral_state import hsr_MoveToNeutralState
from hsr_flexbe_states.hsr_head_move_state import hsr_HeadMoveState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jun 06 2020
@author: YusukeMiake
'''
class hsr_object_search_poseSM(Behavior):
	'''
	Move to object search pose
	'''


	def __init__(self):
		super(hsr_object_search_poseSM, self).__init__()
		self.name = 'hsr_object_search_pose'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:592 y:226, x:459 y:112
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:74 y:104
			OperatableStateMachine.add('MoveToNeutral',
										hsr_MoveToNeutralState(open_hand=False),
										transitions={'succeeded': 'RaiseHead', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:353 y:206
			OperatableStateMachine.add('RaiseHead',
										hsr_HeadMoveState(head_position=0.0),
										transitions={'succeeded': 'finished'},
										autonomy={'succeeded': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
