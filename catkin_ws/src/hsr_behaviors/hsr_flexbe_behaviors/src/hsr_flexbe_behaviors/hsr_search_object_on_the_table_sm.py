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
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jul 12 2019
@author: ShigemichiMatsuzaki
'''
class HSRSearchObjectonthetableSM(Behavior):
	'''
	Test 'search_object' on a short table
	'''


	def __init__(self):
		super(HSRSearchObjectonthetableSM, self).__init__()
		self.name = 'HSR Search Object on the table'

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
			# x:65 y:60
			OperatableStateMachine.add('Search',
										hsr_SearchObjectState(search_point='table', search_place_type='floor', service_search_floor='/search_object/search_floor', service_update_threshold='/ork_tf_broadcaster/update_threshold', centroid_x_max=0.8, centroid_y_max=1.0, centroid_y_min=-1.0, centroid_z_max=0.5, centroid_z_min=0.35, sleep_time=5.0),
										transitions={'found': 'finished', 'notfound': 'finished', 'failed': 'failed'},
										autonomy={'found': Autonomy.Off, 'notfound': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
