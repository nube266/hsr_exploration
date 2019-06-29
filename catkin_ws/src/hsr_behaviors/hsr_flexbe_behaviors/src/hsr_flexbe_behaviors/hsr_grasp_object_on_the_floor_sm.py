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
from hsr_flexbe_states.hsr_fetch_object_state import hsr_FetchObjectState
from hsr_flexbe_states.hsr_pass_object_state import hsr_PassObjectState
from hsr_flexbe_states.hsr_search_object_state import hsr_SearchObjectState
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
										hsr_MoveToNeutralState(),
										transitions={'succeeded': 'SearchObject', 'failed': 'finished'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:454 y:78
			OperatableStateMachine.add('FetchObject',
										hsr_FetchObjectState(fetch_place_type='floor', grasp_srv_name='/grasp/service', stop_tf_srv_name='/ork_tf_broadcaster/stop_publish', target_name='closest'),
										transitions={'succeeded': 'Pass', 'failed': 'MoveToNeutralError'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:289 y:461
			OperatableStateMachine.add('MoveToNeutralError',
										hsr_MoveToNeutralState(),
										transitions={'succeeded': 'failed', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:549 y:301
			OperatableStateMachine.add('Pass',
										hsr_PassObjectState(service_name='/kinesthetic/wait_open'),
										transitions={'succeeded': 'finished', 'failed': 'MoveToNeutralError'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:237 y:53
			OperatableStateMachine.add('SearchObject',
										hsr_SearchObjectState(search_point=self.searching_point, search_place_type='floor', service_name='/search_object/search_floor', centroid_x_max=1.5, centroid_y_max=1.0, centroid_y_min=-1.0, centroid_z_max=0.3, centroid_z_min=0.0),
										transitions={'succeeded': 'FetchObject', 'failed': 'MoveToNeutralError'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
