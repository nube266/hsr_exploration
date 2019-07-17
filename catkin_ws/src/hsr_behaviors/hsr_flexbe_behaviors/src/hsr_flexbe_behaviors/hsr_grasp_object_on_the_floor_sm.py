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
from hsr_flexbe_states.hsr_analyse_command_state import hsr_AnalyseCommandState
from hsr_flexbe_states.hsr_set_base_pose_by_tf_name_dyn_state import hsr_SetBasePoseByTfNameDynState
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBaseState
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

			# x:504 y:243
			OperatableStateMachine.add('FetchObject',
										hsr_FetchObjectState(fetch_place_type='floor', grasp_srv_name='/grasp/service', stop_tf_srv_name='/ork_tf_broadcaster/stop_publish', target_name='closest'),
										transitions={'succeeded': 'SetPosePutPoint', 'failed': 'MoveToNeutralError'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:355 y:349
			OperatableStateMachine.add('MoveToNeutralError',
										hsr_MoveToNeutralState(),
										transitions={'succeeded': 'failed', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:530 y:67
			OperatableStateMachine.add('Analize',
										hsr_AnalyseCommandState(default_location='toyshelf', service_name='/wrs_semantics/tidyup_locationOfObject_stge1'),
										transitions={'succeeded': 'FetchObject', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'command': 'object_name', 'object_name': 'object_name', 'location_name': 'location_name', 'location_to_put': 'location_to_put'})

			# x:578 y:395
			OperatableStateMachine.add('SetPosePutPoint',
										hsr_SetBasePoseByTfNameDynState(service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToPutPoint'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tf_name': 'location_name', 'pose': 'pose'})

			# x:576 y:516
			OperatableStateMachine.add('MoveToPutPoint',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'finished', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:283 y:59
			OperatableStateMachine.add('SearchObject',
										hsr_SearchObjectState(search_point='search_point_0', search_place_type='floor', service_search_floor='/search_object/search_floor', service_update_threshold='/ork_tf_broadcaster/update_threshold', service_publish_tf='/ork_tf_broadcaster/start_publish', service_stop_tf='/ork_tf_broadcaster/stop_publish', centroid_x_max=1.5, centroid_y_max=1.0, centroid_y_min=-1.0, centroid_z_max=0.2, centroid_z_min=0.0, sleep_time=5.0, is_floor=False),
										transitions={'found': 'Analize', 'notfound': 'finished', 'failed': 'failed'},
										autonomy={'found': Autonomy.Off, 'notfound': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_name': 'object_name'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
