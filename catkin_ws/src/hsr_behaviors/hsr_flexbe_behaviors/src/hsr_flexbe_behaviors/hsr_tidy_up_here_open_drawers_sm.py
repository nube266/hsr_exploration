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
from hsr_flexbe_states.hsr_open_drawer_state import hsr_OpenDrawerState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jul 25 2019
@author: ShigemichiMatsuzaki
'''
class HSRTidyUpHereOpenDrawersSM(Behavior):
	'''
	Behavior to open the three drawers
	'''


	def __init__(self):
		super(HSRTidyUpHereOpenDrawersSM, self).__init__()
		self.name = 'HSR Tidy Up Here Open Drawers'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:283 y:477, x:130 y:478
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('SetPose1',
										hsr_SetBasePoseByTfNameState(tf_name='drawer1', service_name='/pose_server/getPose'),
										transitions={'completed': 'Move1'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:30 y:117
			OperatableStateMachine.add('Move1',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'OpenDrawer1', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:30 y:209
			OperatableStateMachine.add('OpenDrawer1',
										hsr_OpenDrawerState(open_srv_name='/grasp/open_drawer', height=0),
										transitions={'succeeded': 'SetPose2', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:218 y:40
			OperatableStateMachine.add('SetPose2',
										hsr_SetBasePoseByTfNameState(tf_name='drawer2', service_name='/pose_server/getPose'),
										transitions={'completed': 'Move2'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:219 y:117
			OperatableStateMachine.add('Move2',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'OpenDrawer2', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:217 y:212
			OperatableStateMachine.add('OpenDrawer2',
										hsr_OpenDrawerState(open_srv_name='/grasp/open_drawer', height=0),
										transitions={'succeeded': 'SetPose3', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:437 y:38
			OperatableStateMachine.add('SetPose3',
										hsr_SetBasePoseByTfNameState(tf_name='drawer2', service_name='/pose_server/getPose'),
										transitions={'completed': 'Move3'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:438 y:118
			OperatableStateMachine.add('Move3',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'OpenDrawer3', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:432 y:220
			OperatableStateMachine.add('OpenDrawer3',
										hsr_OpenDrawerState(open_srv_name='/grasp/open_drawer', height=1),
										transitions={'succeeded': 'finished', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
