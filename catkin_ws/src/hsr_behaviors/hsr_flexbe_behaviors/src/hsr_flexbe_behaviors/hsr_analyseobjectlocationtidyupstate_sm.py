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
class HSRAnalyseObjectLocationTidyUpStateSM(Behavior):
	'''
	Test new analysis state
	'''


	def __init__(self):
		super(HSRAnalyseObjectLocationTidyUpStateSM, self).__init__()
		self.name = 'HSR AnalyseObjectLocationTidyUpState'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:311 y:423
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['command'])
		_state_machine.userdata.command = 'masterchefcan'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:93 y:77
			OperatableStateMachine.add('Analyse',
										hsr_AnalyseObjectLocationTidyUpState(default_location='bin', default_deposit='binb', service_name='/wrs_semantics/tidyup_locationAndDepositOfObject_task1'),
										transitions={'succeeded': 'Speak', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'command': 'command', 'location_name': 'location_name', 'location_to_put': 'location_to_put'})

			# x:150 y:172
			OperatableStateMachine.add('Speak',
										hsr_SpeakDynState(sentence="location name is +", sentence_when_empty="I couldn't recognizeit", topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'Speak1', 'empty': 'Speak1'},
										autonomy={'done': Autonomy.Off, 'empty': Autonomy.Off},
										remapping={'variable': 'location_name'})

			# x:150 y:285
			OperatableStateMachine.add('Speak1',
										hsr_SpeakDynState(sentence="deposit place name is +", sentence_when_empty="I couldn't recognizeit", topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'finished', 'empty': 'finished'},
										autonomy={'done': Autonomy.Off, 'empty': Autonomy.Off},
										remapping={'variable': 'location_to_put'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
