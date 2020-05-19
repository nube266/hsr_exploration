#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_speak_dyn_state import hsr_SpeakDynState
from flexbe_states.wait_state import WaitState
from hsr_flexbe_states.hsr_move_to_neutral_state import hsr_MoveToNeutralState
from hsr_flexbe_states.hsr_speak_state import hsr_SpeakState
from flexbe_states.subscriber_state import SubscriberState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Aug 20 2019
@author: ShigemichiMatsuzaki
'''
class HSRQRSpeakSM(Behavior):
	'''
	Demo to make HSR speak
	'''


	def __init__(self):
		super(HSRQRSpeakSM, self).__init__()
		self.name = 'HSR QR Speak'

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
			# x:184 y:165
			OperatableStateMachine.add('Neutral',
										hsr_MoveToNeutralState(open_hand=False),
										transitions={'succeeded': 'SpeakExplain', 'failed': 'SpeakExplain'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:349 y:248
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=2.0),
										transitions={'done': 'SubscribeString'},
										autonomy={'done': Autonomy.Off})

			# x:341 y:58
			OperatableStateMachine.add('SpeakExplain',
										hsr_SpeakState(sentence='qrコードを見せてね', topic='/talk_request', interrupting=False, queueing=False, language=0),
										transitions={'done': 'SubscribeString'},
										autonomy={'done': Autonomy.Off})

			# x:554 y:109
			OperatableStateMachine.add('SubscribeString',
										SubscriberState(topic='/barcode', blocking=True, clear=True),
										transitions={'received': 'Speak', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:351 y:173
			OperatableStateMachine.add('Speak',
										hsr_SpeakDynState(sentence="+", sentence_when_empty='', topic='/talk_request', interrupting=False, queueing=False, language=0),
										transitions={'done': 'Wait', 'empty': 'SubscribeString'},
										autonomy={'done': Autonomy.Off, 'empty': Autonomy.Off},
										remapping={'variable': 'message'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
