#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_search_object_dyn_state import hsr_SearchObjectDynState
from hsr_flexbe_states.hsr_fetch_object_state import hsr_FetchObjectState
from hsr_flexbe_states.hsr_analyse_object_location_tidyup_state import hsr_AnalyseObjectLocationTidyUpState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Aug 16 2019
@author: ShigemichiMatsuzaki
'''
class HSRFetchObjectNewSM(Behavior):
	'''
	Behavior of the object fetching process
Search -> Fetch -> Analyse
	'''


	def __init__(self):
		super(HSRFetchObjectNewSM, self).__init__()
		self.name = 'HSR FetchObjectNew'

		# parameters of this behavior
		self.add_parameter('time_limit', 5.0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1002 y:396, x:467 y:457, x:230 y:478, x:316 y:477
		_state_machine = OperatableStateMachine(outcomes=['finished', 'grasp_failed', 'not_found', 'failed'], input_keys=['search_centroid_y_max', 'search_centroid_y_min', 'search_centroid_z_max', 'search_centroid_z_min', 'search_sleep_time', 'search_is_floor', 'search_centroid_x_max'], output_keys=['object_name', 'location_name', 'location_to_put'])
		_state_machine.userdata.search_centroid_x_max = 1.5
		_state_machine.userdata.search_centroid_y_max = 0.5
		_state_machine.userdata.search_centroid_y_min = -0.5
		_state_machine.userdata.search_centroid_z_max = 0.2
		_state_machine.userdata.search_centroid_z_min = 0.0
		_state_machine.userdata.search_sleep_time = 5.0
		_state_machine.userdata.search_is_floor = False
		_state_machine.userdata.object_name = ''
		_state_machine.userdata.location_name = ''
		_state_machine.userdata.location_to_put = ''

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:160 y:72
			OperatableStateMachine.add('SearchObject',
										hsr_SearchObjectDynState(search_point='', search_place_type='floor', service_search_floor='/search_object/search_floor', service_update_threshold='/ork_tf_broadcaster/update_threshold', service_publish_tf='/ork_tf_broadcaster/start_publish', service_stop_tf='/ork_tf_broadcaster/stop_publish'),
										transitions={'found': 'FetchObject', 'notfound': 'not_found', 'failed': 'failed'},
										autonomy={'found': Autonomy.Off, 'notfound': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'centroid_x_max': 'search_centroid_x_max', 'centroid_y_max': 'search_centroid_y_max', 'centroid_y_min': 'search_centroid_y_min', 'centroid_z_max': 'search_centroid_z_max', 'centroid_z_min': 'search_centroid_z_min', 'sleep_time': 'search_sleep_time', 'is_floor': 'search_is_floor', 'object_name': 'object_name'})

			# x:527 y:89
			OperatableStateMachine.add('FetchObject',
										hsr_FetchObjectState(fetch_place_type='floor', grasp_srv_name='/grasp/service', stop_tf_srv_name='/ork_tf_broadcaster/stop_publish', target_name='closest'),
										transitions={'succeeded': 'Analyze', 'failed': 'grasp_failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:670 y:200
			OperatableStateMachine.add('Analyze',
										hsr_AnalyseObjectLocationTidyUpState(default_location='bin', default_deposit='binb', service_name='/wrs_semantics/tidyup_locationAndDepositOfObject_task1'),
										transitions={'succeeded': 'finished', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'command': 'object_name', 'location_name': 'location_name', 'location_to_put': 'location_to_put'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
