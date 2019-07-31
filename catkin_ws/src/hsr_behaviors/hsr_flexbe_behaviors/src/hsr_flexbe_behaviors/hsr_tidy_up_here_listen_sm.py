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
from flexbe_states.subscriber_state import SubscriberState
from hsr_flexbe_states.hsr_speak_dyn_state import hsr_SpeakDynState
from hsr_flexbe_states.hsr_analyse_command_state import hsr_AnalyseCommandState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jul 26 2019
@author: ShigemichiMatsuzaki
'''
class HSRTidyUpHereListenSM(Behavior):
	'''
	Listening behavior
	'''


	def __init__(self):
		super(HSRTidyUpHereListenSM, self).__init__()
		self.name = 'HSR Tidy Up Here Listen'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:143 y:541, x:467 y:343
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['object_name'])
		_state_machine.userdata.object_name = ''

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('SpeakGiveMe',
										hsr_SpeakState(sentence='How may I help you?', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'Wait1'},
										autonomy={'done': Autonomy.Off})

			# x:243 y:114
			OperatableStateMachine.add('ListenObject',
										SubscriberState(topic='/sr_res', blocking=True, clear=True),
										transitions={'received': 'Analyse', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:209 y:217
			OperatableStateMachine.add('SpeakObject',
										hsr_SpeakDynState(sentence="I'll bring +", sentence_when_empty="Could you say that again?", topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'Wait2', 'empty': 'Wait1'},
										autonomy={'done': Autonomy.Off, 'empty': Autonomy.Off},
										remapping={'variable': 'object_name'})

			# x:100 y:409
			OperatableStateMachine.add('ListenTidyUp',
										SubscriberState(topic='/sr_res', blocking=True, clear=True),
										transitions={'received': 'finished', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:441 y:102
			OperatableStateMachine.add('Analyse',
										hsr_AnalyseCommandState(default_location='shelf', default_id='0', service_name='/wrs_semantics_bring/bring_me_instruction'),
										transitions={'succeeded': 'SpeakObject', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'command': 'message', 'object_name': 'object_name', 'location_name': 'location_name', 'location_to_put': 'location_to_put'})

			# x:119 y:102
			OperatableStateMachine.add('Wait1',
										WaitState(wait_time=1.5),
										transitions={'done': 'ListenObject'},
										autonomy={'done': Autonomy.Off})

			# x:122 y:285
			OperatableStateMachine.add('Wait2',
										WaitState(wait_time=1.5),
										transitions={'done': 'ListenTidyUp'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
