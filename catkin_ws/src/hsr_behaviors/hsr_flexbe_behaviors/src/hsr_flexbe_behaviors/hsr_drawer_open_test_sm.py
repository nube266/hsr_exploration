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
from hsr_flexbe_states.hsr_open_drawer_state import hsr_OpenDrawerState
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBaseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jul 18 2019
@author: ShigemichiMatsuzaki
'''
class HSRDrawerOpenTestSM(Behavior):
	'''
	Test behavior for 'hsr_OpenDrawerState'
	'''


	def __init__(self):
		super(HSRDrawerOpenTestSM, self).__init__()
		self.name = 'HSR Drawer Open Test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('SetPose',
										hsr_SetBasePoseByTfNameState(tf_name='toyshelf', service_name='/pose_server/getPose'),
										transitions={'completed': 'Move'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:27 y:177
			OperatableStateMachine.add('OpenDrawer',
										hsr_OpenDrawerState(open_srv_name='/grasp/open_drawer', height=0),
										transitions={'succeeded': 'finished', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:218 y:85
			OperatableStateMachine.add('Move',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'OpenDrawer', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
