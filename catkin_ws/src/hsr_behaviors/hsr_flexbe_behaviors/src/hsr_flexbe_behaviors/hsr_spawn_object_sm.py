#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_spawn_model_state import hsr_SpawnModelState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Jul 26 2020
@author: Yusuke Miake
'''
class hsr_spawn_objectSM(Behavior):
	'''
	The behavior that causes the object to spawn
	'''


	def __init__(self):
		super(hsr_spawn_objectSM, self).__init__()
		self.name = 'hsr_spawn_object'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:395 y:170, x:365 y:253
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:101 y:160
			OperatableStateMachine.add('SpawnModel',
										hsr_SpawnModelState(model_path="/root/HSR/catkin_ws/src/hsr_object_search_world/models/book_1/model.sdf", model_name="book_1", model_format="sdf", x=1.0, y=1.0, z=0.0),
										transitions={'succeeded': 'finished'},
										autonomy={'succeeded': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
