#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_move_to_neutral_state import hsr_MoveToNeutralState
from hsr_flexbe_behaviors.hsr_fetchobject_sm import HSRFetchObjectSM
from hsr_flexbe_states.hsr_fetch_object_interface_state import hsr_FetchObjectInterfaceState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jun 07 2019
@author: ShigemichiMatsuzaki
'''
class HSRgraspobjectonthefloorSM(Behavior):
	'''
	Demo code for the task of recognizing and grasping an object on the floor
	'''


	def __init__(self):
		super(HSRgraspobjectonthefloorSM, self).__init__()
		self.name = 'HSR grasp object on the floor'

		# parameters of this behavior
		self.add_parameter('searching_point', 'searching_point_0')

		# references to used behaviors
		self.add_behavior(HSRFetchObjectSM, 'HSR FetchObject')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:106 y:591
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:45 y:51
			OperatableStateMachine.add('MoveToNeutral',
										hsr_MoveToNeutralState(open_hand=True),
										transitions={'succeeded': 'Interface', 'failed': 'finished'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:492 y:203
			OperatableStateMachine.add('HSR FetchObject',
										self.use_behavior(HSRFetchObjectSM, 'HSR FetchObject'),
										transitions={'finished': 'finished', 'grasp_failed': 'failed', 'not_found': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'grasp_failed': Autonomy.Inherit, 'not_found': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'search_centroid_y_max': 'centroid_y_max', 'search_centroid_y_min': 'centroid_y_min', 'search_centroid_z_max': 'centroid_z_max', 'search_centroid_z_min': 'centroid_z_min', 'search_sleep_time': 'sleep_time', 'search_is_floor': 'is_floor', 'search_centroid_x_max': 'centroid_x_max', 'object_name': 'object_name', 'location_name': 'location_name', 'location_to_put': 'location_to_put'})

			# x:217 y:43
			OperatableStateMachine.add('Interface',
										hsr_FetchObjectInterfaceState(centroid_x_max=1.5, centroid_y_max=1.0, centroid_y_min=-1.0, centroid_z_max=0.3, centroid_z_min=0.0, sleep_time=8.0, is_floor=False),
										transitions={'done': 'HSR FetchObject'},
										autonomy={'done': Autonomy.Off},
										remapping={'centroid_x_max': 'centroid_x_max', 'centroid_y_max': 'centroid_y_max', 'centroid_y_min': 'centroid_y_min', 'centroid_z_max': 'centroid_z_max', 'centroid_z_min': 'centroid_z_min', 'sleep_time': 'sleep_time', 'is_floor': 'is_floor'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
