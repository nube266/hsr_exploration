#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_speak_state import hsr_SpeakState
from flexbe_states.decision_state import DecisionState
from flexbe_states.wait_state import WaitState
from flexbe_states.subscriber_state import SubscriberState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Aug 16 2019
@author: ShigemichiMatsuzaki
'''
class HSRWaitfordooropenedSM(Behavior):
	'''
	A behavior for the robot inspection task in RoboCup Japan Open 2019
	'''


	def __init__(self):
		super(HSRWaitfordooropenedSM, self).__init__()
		self.name = 'HSR Wait for door opened'

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
			# x:281 y:28
			OperatableStateMachine.add('SpeakStart',
										hsr_SpeakState(sentence="Please open the door", topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'Subscribe'},
										autonomy={'done': Autonomy.Off})

			# x:695 y:147
			OperatableStateMachine.add('D',
										DecisionState(outcomes=['is_person', 'no_person'], conditions=lambda x : 'is_person' if x.data else 'no_person'),
										transitions={'is_person': 'Subscribe', 'no_person': 'Speak'},
										autonomy={'is_person': Autonomy.Off, 'no_person': Autonomy.Off},
										remapping={'input_value': 'message'})

			# x:408 y:322
			OperatableStateMachine.add('W',
										WaitState(wait_time=5.0),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:617 y:262
			OperatableStateMachine.add('Speak',
										hsr_SpeakState(sentence="Door opened. I'm moving after 5 seconds", topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'W'},
										autonomy={'done': Autonomy.Off})

			# x:495 y:72
			OperatableStateMachine.add('Subscribe',
										SubscriberState(topic='/ork_door_detector/is_door', blocking=True, clear=True),
										transitions={'received': 'D', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
