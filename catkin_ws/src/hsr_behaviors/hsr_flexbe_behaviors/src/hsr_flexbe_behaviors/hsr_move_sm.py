#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_set_base_pose_by_tf_name_dyn_state import hsr_SetBasePoseByTfNameDynState
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBaseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jul 22 2019
@author: ShigemichiMatsuzaki
'''
class HSRMoveSM(Behavior):
	'''
	State to set a pose and call MoveBase action
	'''


	def __init__(self):
		super(HSRMoveSM, self).__init__()
		self.name = 'HSR Move'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:478, x:240 y:463
		_state_machine = OperatableStateMachine(outcomes=['succeeded', 'failed'], input_keys=['tf_name'])
		_state_machine.userdata.tf_name = 'searching_point_0'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:59 y:77
			OperatableStateMachine.add('SetPose',
										hsr_SetBasePoseByTfNameDynState(service_name='/pose_server/getPose'),
										transitions={'completed': 'Move'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tf_name': 'tf_name', 'pose': 'pose'})

			# x:187 y:268
			OperatableStateMachine.add('Move',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'succeeded', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
