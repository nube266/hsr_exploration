#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_behaviors.hsr_setup_for_experiment_sm import hsr_setup_for_experimentSM
from hsr_flexbe_behaviors.hsr_object_search_simulation_sm import hsr_object_search_simulationSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 29 2020
@author: YusukeMiake
'''
class ExperimentRSJSM(Behavior):
	'''
	For RSJ
	'''


	def __init__(self):
		super(ExperimentRSJSM, self).__init__()
		self.name = 'ExperimentRSJ'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(hsr_setup_for_experimentSM, 'hsr_setup_for_experiment')
		self.add_behavior(hsr_object_search_simulationSM, 'hsr_object_search_simulation')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:501 y:315, x:182 y:316
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:127 y:118
			OperatableStateMachine.add('hsr_setup_for_experiment',
										self.use_behavior(hsr_setup_for_experimentSM, 'hsr_setup_for_experiment'),
										transitions={'finished': 'hsr_object_search_simulation', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:416 y:122
			OperatableStateMachine.add('hsr_object_search_simulation',
										self.use_behavior(hsr_object_search_simulationSM, 'hsr_object_search_simulation'),
										transitions={'finished': 'hsr_setup_for_experiment', 'failed': 'finished'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
