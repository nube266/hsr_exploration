#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_behaviors.hsr_task2a_sm import HSRTask2aSM
from hsr_flexbe_states.hsr_set_base_pose_by_tf_name_state import hsr_SetBasePoseByTfNameState
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBaseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Aug 07 2019
@author: ShigemichiMatsuzaki
'''
class HSRTidyUpHereTask2aSM(Behavior):
	'''
	Behavior of Tidy Up Here Task 2a in Robocup 2019/WRS2020, where the robot has to navigate avoiding small objects.
	'''


	def __init__(self):
		super(HSRTidyUpHereTask2aSM, self).__init__()
		self.name = 'HSR Tidy Up Here Task 2a'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(HSRTask2aSM, 'HSR Task2a')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:478, x:130 y:478
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('SetPose',
										hsr_SetBasePoseByTfNameState(tf_name='task2a_neutral', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToNeutralPosition'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:279 y:110
			OperatableStateMachine.add('MoveToNeutralPosition',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'HSR Task2a', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:323 y:264
			OperatableStateMachine.add('HSR Task2a',
										self.use_behavior(HSRTask2aSM, 'HSR Task2a'),
										transitions={'finished': 'finished', 'failed': 'SetPose'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
