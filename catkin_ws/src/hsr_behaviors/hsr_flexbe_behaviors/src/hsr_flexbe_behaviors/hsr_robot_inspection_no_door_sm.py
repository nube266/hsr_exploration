#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_wait_press_wrist_state import hsr_WaitWristPressedState
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBaseState
from hsr_flexbe_states.hsr_speak_state import hsr_SpeakState
from hsr_flexbe_states.hsr_set_base_pose_by_tf_name_state import hsr_SetBasePoseByTfNameState
from flexbe_states.wait_state import WaitState
from hsr_flexbe_states.hsr_escape_by_twist_state import hsr_EscapeByTwistState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Aug 05 2019
@author: ShigemichiMatsuzaki
'''
class HSRRobotInspectionnodoorSM(Behavior):
	'''
	A behavior for the robot inspection task in RoboCup Japan Open 2019
	'''


	def __init__(self):
		super(HSRRobotInspectionnodoorSM, self).__init__()
		self.name = 'HSR Robot Inspection no door'

		# parameters of this behavior

		# references to used behaviors

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
			# x:23 y:59
			OperatableStateMachine.add('Start',
										hsr_WaitWristPressedState(),
										transitions={'succeeded': 'Wait'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'start_time': 'start_time'})

			# x:146 y:252
			OperatableStateMachine.add('MoveToArenaGoal',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'Speak', 'failed': 'Twist2'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:73 y:343
			OperatableStateMachine.add('Speak',
										hsr_SpeakState(sentence='I am here!', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:100 y:175
			OperatableStateMachine.add('SetPoseArenaGoal',
										hsr_SetBasePoseByTfNameState(tf_name='arena_goal', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToArenaGoal'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:607 y:162
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=5.0),
										transitions={'done': 'SpeakMove'},
										autonomy={'done': Autonomy.Off})

			# x:305 y:205
			OperatableStateMachine.add('Twist2',
										hsr_EscapeByTwistState(topic='/hsrb/command_velocity', linear_x=0, linear_y=0.0, angular=0.5, duration=1.5),
										transitions={'completed': 'SetPoseArenaGoal'},
										autonomy={'completed': Autonomy.Off})

			# x:470 y:187
			OperatableStateMachine.add('SpeakMove',
										hsr_SpeakState(sentence='I am moving', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'SetPoseArenaGoal'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
