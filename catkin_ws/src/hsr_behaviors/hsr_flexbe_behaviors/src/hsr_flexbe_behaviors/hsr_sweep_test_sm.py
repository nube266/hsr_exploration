#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_set_base_pose_by_tf_name_state import hsr_SetBasePoseByTfNameState
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBaseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jun 19 2019
@author: YusukeMiake
'''
class HSRsweeptestSM(Behavior):
	'''
	Move to the destination.
Sweep if there is an obstacle at the destination.
	'''


	def __init__(self):
		super(HSRsweeptestSM, self).__init__()
		self.name = 'HSR sweep test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:410 y:393, x:247 y:398
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:43 y:108
			OperatableStateMachine.add('SetPoseDestination',
										hsr_SetBasePoseByTfNameState(tf_name='searching_point_0', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToDestination'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:283 y:226
			OperatableStateMachine.add('MoveToDestination',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'finished', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
