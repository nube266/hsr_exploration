#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_search_object_state import hsr_SearchObjectState
from hsr_flexbe_states.hsr_sweep_object_state import hsr_SweepObjectState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jun 19 2019
@author: YusukeMiake
'''
class HSRsweeptestSM(Behavior):
	'''
	Sweep "cloest" object.
	'''


	def __init__(self):
		super(HSRsweeptestSM, self).__init__()
		self.name = 'HSR sweep test'

		# parameters of this behavior
		self.add_parameter('searching_point', 'searching_point_0')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:594 y:136, x:108 y:423
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:78 y:129
			OperatableStateMachine.add('SearchClosestObject',
										hsr_SearchObjectState(search_point=self.searching_point, search_place_type='floor', service_name='/search_object/search_floor', centroid_x_max=1.5, centroid_y_max=1.0, centroid_y_min=-1.0, centroid_z_max=0.3, centroid_z_min=0.0),
										transitions={'succeeded': 'SweepClosestObject', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:325 y:124
			OperatableStateMachine.add('SweepClosestObject',
										hsr_SweepObjectState(target_name='closest', sweep_place_type='floor', sweep_srv_name='/sweep', waiting_time=0, sweep_mode='push', sweep_angular=1, sweep_distance=0.25, sweep_height=0.0, stop_tf_srv_name='/ork_tf_broadcaster/stop_publish'),
										transitions={'succeeded': 'finished', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
