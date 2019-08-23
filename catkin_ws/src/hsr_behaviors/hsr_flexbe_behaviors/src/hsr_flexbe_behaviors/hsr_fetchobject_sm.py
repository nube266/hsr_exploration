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
from hsr_flexbe_states.hsr_analyse_command_state import hsr_AnalyseCommandState
from hsr_flexbe_states.hsr_speak_state import hsr_SpeakState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed July 20 2019
@author: ShigemichiMatsuzaki
'''
class HSRFetchObjectSM(Behavior):
	'''
	Behavior of the object fetching process
Search -> Fetch -> Analyse
	'''


	def __init__(self):
		super(HSRFetchObjectSM, self).__init__()
		self.name = 'HSR FetchObject'

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
		_state_machine.userdata.search_centroid_x_max = 1.0
		_state_machine.userdata.search_centroid_y_max = 0.25
		_state_machine.userdata.search_centroid_y_min = -0.25
		_state_machine.userdata.search_centroid_z_max = 0.80
		_state_machine.userdata.search_centroid_z_min = 0.61
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
										transitions={'found': 'Speak', 'notfound': 'not_found', 'failed': 'failed'},
										autonomy={'found': Autonomy.Off, 'notfound': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'centroid_x_max': 'search_centroid_x_max', 'centroid_y_max': 'search_centroid_y_max', 'centroid_y_min': 'search_centroid_y_min', 'centroid_z_max': 'search_centroid_z_max', 'centroid_z_min': 'search_centroid_z_min', 'sleep_time': 'search_sleep_time', 'is_floor': 'search_is_floor', 'object_name': 'object_name'})

			# x:476 y:119
			OperatableStateMachine.add('FetchObject',
										hsr_FetchObjectState(fetch_place_type='floor', grasp_srv_name='/grasp/service', stop_tf_srv_name='/ork_tf_broadcaster/stop_publish', target_name='closest'),
										transitions={'succeeded': 'Analyse', 'failed': 'grasp_failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:652 y:178
			OperatableStateMachine.add('Analyse',
										hsr_AnalyseCommandState(default_location='bin', default_id=0, service_name='/wrs_semantics/tidyup_locationOfObject_stge1'),
										transitions={'succeeded': 'finished', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'command': 'object_name', 'object_name': 'object_name', 'location_name': 'location_name', 'location_to_put': 'location_to_put'})

			# x:368 y:68
			OperatableStateMachine.add('Speak',
										hsr_SpeakState(sentence='見つけた！', topic='/talk_request', interrupting=False, queueing=False, language=0),
										transitions={'done': 'FetchObject'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
