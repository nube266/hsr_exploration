#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_escape_by_twist_state import hsr_EscapeByTwistState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jun 08 2019
@author: ShigemichiMatsuzaki
'''
class HSRescapetestSM(Behavior):
	'''
	Test code of hsr_EscapeByTwistState
	'''


	def __init__(self):
		super(HSRescapetestSM, self).__init__()
		self.name = 'HSR escape test'

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
			# x:166 y:189
			OperatableStateMachine.add('Escape',
										hsr_EscapeByTwistState(topic='/hsrb/command_velocity', linear_x=-0.0, linear_y=0.0, angular=3.14, duration=1),
										transitions={'completed': 'finished'},
										autonomy={'completed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
