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
from hsr_flexbe_states.hsr_escape_by_twist_state import hsr_EscapeByTwistState
from flexbe_states.subscriber_state import SubscriberState
from flexbe_states.decision_state import DecisionState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Aug 05 2019
@author: ShigemichiMatsuzaki
'''
class HSRRobotInspectionSM(Behavior):
	'''
	A behavior for the robot inspection task in RoboCup Japan Open 2019
	'''


	def __init__(self):
		super(HSRRobotInspectionSM, self).__init__()
		self.name = 'HSR Robot Inspection'

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
			# x:57 y:30
			OperatableStateMachine.add('SpeakImReady',
										hsr_SpeakState(sentence="I'm ready", topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'Start'},
										autonomy={'done': Autonomy.Off})

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

			# x:442 y:188
			OperatableStateMachine.add('SetPoseArenaGoal',
										hsr_SetBasePoseByTfNameState(tf_name='arena_goal', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToArenaGoal'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:294 y:205
			OperatableStateMachine.add('Twist2',
										hsr_EscapeByTwistState(topic='/hsrb/command_velocity', linear_x=0, linear_y=0.0, angular=0.5, duration=1.5),
										transitions={'completed': 'SetPoseArenaGoal'},
										autonomy={'completed': Autonomy.Off})

			# x:560 y:78
			OperatableStateMachine.add('Subscribe',
										SubscriberState(topic='/ork_door_detector/is_door', blocking=True, clear=True),
										transitions={'received': 'D', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:760 y:62
			OperatableStateMachine.add('D',
										DecisionState(outcomes=['is_person', 'no_person'], conditions=lambda x : 'is_person' if x.data else 'no_person'),
										transitions={'is_person': 'Subscribe', 'no_person': 'W'},
										autonomy={'is_person': Autonomy.Off, 'no_person': Autonomy.Off},
										remapping={'input_value': 'message'})

			# x:668 y:212
			OperatableStateMachine.add('W',
										WaitState(wait_time=5.0),
										transitions={'done': 'SetPoseArenaGoal'},
										autonomy={'done': Autonomy.Off})

			# x:432 y:24
			OperatableStateMachine.add('SpeakStart',
										hsr_SpeakState(sentence="Let's start", topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'Subscribe'},
										autonomy={'done': Autonomy.Off})

			# x:222 y:20
			OperatableStateMachine.add('Start',
										hsr_WaitWristPressedState(),
										transitions={'succeeded': 'SpeakStart'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'start_time': 'start_time'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
