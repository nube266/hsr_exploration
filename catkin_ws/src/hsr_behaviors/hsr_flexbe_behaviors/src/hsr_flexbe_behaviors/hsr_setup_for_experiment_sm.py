#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_set_pose_state import hsr_SetBasePoseState
from hsr_flexbe_states.hsr_reset_octomap_state import hsr_ResetOctomapState
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBaseState
from hsr_flexbe_states.hsr_restart_gmapping_state import hsr_RestartGmappingState
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
		# x:602 y:276, x:308 y:167
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:87 y:51
			OperatableStateMachine.add('SetInitialPose',
										hsr_SetBasePoseState(x=0.0, y=0.0, yaw=0.0),
										transitions={'succeeded': 'MoveToInitialPose'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:87 y:275
			OperatableStateMachine.add('ResetOctomap',
										hsr_ResetOctomapState(srv_name="/octomap_server/reset"),
										transitions={'succeeded': 'ResetMap'},
										autonomy={'succeeded': Autonomy.Off})

			# x:91 y:157
			OperatableStateMachine.add('MoveToInitialPose',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'ResetOctomap', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:328 y:268
			OperatableStateMachine.add('ResetMap',
										hsr_RestartGmappingState(),
										transitions={'succeeded': 'finished'},
										autonomy={'succeeded': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
