#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_analyse_object_location_tidyup_state import hsr_AnalyseObjectLocationTidyUpState
from hsr_flexbe_states.hsr_speak_dyn_state import hsr_SpeakDynState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Aug 16 2019
@author: ShigemichiMatsuzaki
'''
class HSRTestAnalyseObjectLocationTidyUpStateSM(Behavior):
	'''
	Test 'hsr_AnalyseObjectLocationTidyUpState'
	'''


	def __init__(self):
		super(HSRTestAnalyseObjectLocationTidyUpStateSM, self).__init__()
		self.name = 'HSR Test AnalyseObjectLocationTidyUpState'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['command'])
		_state_machine.userdata.command = 'woodblock'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:77 y:78
			OperatableStateMachine.add('Analyze',
										hsr_AnalyseObjectLocationTidyUpState(service_name='/wrs_semantics/tidyup_locationAndDepositOfObject_task1'),
										transitions={'succeeded': 'Speak', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'command': 'command', 'location_name': 'location_name', 'deposit_name': 'deposit_name'})

			# x:232 y:208
			OperatableStateMachine.add('Speak',
										hsr_SpeakDynState(sentence="It's in +", sentence_when_empty="I couldn't recognize it", topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'finished', 'empty': 'finished'},
										autonomy={'done': Autonomy.Off, 'empty': Autonomy.Off},
										remapping={'variable': 'location_name'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
