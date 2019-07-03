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
from hsr_flexbe_states.hsr_speak_state import hsr_SpeakState
from hsr_flexbe_behaviors.hsr_sweep_test_sm import HSRsweeptestSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed July 3 2019
@author: ShigemichiMatsuzaki
'''
class HSRmoveandsweepSM(Behavior):
	'''
	Test code of moving and sweeping after the moving has failed
	'''


	def __init__(self):
		super(HSRmoveandsweepSM, self).__init__()
		self.name = 'HSR move and sweep'

		# parameters of this behavior
		self.add_parameter('searching_point', 'searching_point_0')

		# references to used behaviors
		self.add_behavior(HSRsweeptestSM, 'HSR sweep test')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:662 y:115, x:108 y:423
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('SetPoseSearchPoint',
										hsr_SetBasePoseByTfNameState(tf_name='searching_point_1', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveBase'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:112 y:141
			OperatableStateMachine.add('MoveBase',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'finished', 'failed': 'Speak'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:222 y:203
			OperatableStateMachine.add('Speak',
										hsr_SpeakState(sentence='failed to move. Lets recover', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'SetPoseRecoveryPoint'},
										autonomy={'done': Autonomy.Off})

			# x:376 y:204
			OperatableStateMachine.add('SetPoseRecoveryPoint',
										hsr_SetBasePoseByTfNameState(tf_name='recovery_point', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToRecoveryPoint'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:468 y:322
			OperatableStateMachine.add('MoveToRecoveryPoint',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'HSR sweep test', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:385 y:495
			OperatableStateMachine.add('HSR sweep test',
										self.use_behavior(HSRsweeptestSM, 'HSR sweep test'),
										transitions={'finished': 'SetPoseSearchPoint', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
