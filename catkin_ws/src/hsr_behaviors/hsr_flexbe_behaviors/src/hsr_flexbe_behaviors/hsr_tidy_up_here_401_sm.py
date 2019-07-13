#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_speak_state import hsr_SpeakState
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBaseState
from hsr_flexbe_states.hsr_set_base_pose_by_tf_name_state import hsr_SetBasePoseByTfNameState
from hsr_flexbe_states.hsr_move_to_neutral_state import hsr_MoveToNeutralState
from hsr_flexbe_states.hsr_search_object_state import hsr_SearchObjectState
from hsr_flexbe_behaviors.hsr_sweep_test_sm import HSRsweeptestSM
from hsr_flexbe_states.hsr_analyse_command_state import hsr_AnalyseCommandState
from hsr_flexbe_states.hsr_set_base_pose_by_tf_name_dyn_state import hsr_SetBasePoseByTfNameDynState
from hsr_flexbe_states.hsr_put_object_dyn_state import hsr_PutObjectDynState
from hsr_flexbe_states.hsr_fetch_object_state import hsr_FetchObjectState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed July 02 2019
@author: ShigemichiMatsuzaki
'''
class HSRTidyUpHere401SM(Behavior):
	'''
	Demo code of the entire flow of Tidy Up Here in Sokento 401
	'''


	def __init__(self):
		super(HSRTidyUpHere401SM, self).__init__()
		self.name = 'HSR Tidy Up Here 401'

		# parameters of this behavior
		self.add_parameter('searching_point', 'searching_point_0')
		self.add_parameter('putting_point', 'rubbishbin')

		# references to used behaviors
		self.add_behavior(HSRsweeptestSM, 'HSR sweep test')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:87 y:660, x:1100 y:375
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:254 y:60
			OperatableStateMachine.add('Speak',
										hsr_SpeakState(sentence='I am going to tidy up here', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'SetPoseSearchingPoint'},
										autonomy={'done': Autonomy.Off})

			# x:561 y:505
			OperatableStateMachine.add('MoveToPuttingPoint',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'PutDyn', 'failed': 'SetPoseRecoveryPoint'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:74 y:156
			OperatableStateMachine.add('SetPoseSearchingPoint',
										hsr_SetBasePoseByTfNameState(tf_name='searching_point_1', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToSearchingPoint'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:709 y:237
			OperatableStateMachine.add('SpeakErrorMoveBase',
										hsr_SpeakState(sentence='Failed to move', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:422 y:148
			OperatableStateMachine.add('MoveToSearchingPoint',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'SearchObject', 'failed': 'SpeakErrorMoveBase'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:24 y:369
			OperatableStateMachine.add('MoveToNeutral2',
										hsr_MoveToNeutralState(),
										transitions={'succeeded': 'SetPoseSearchingPoint', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:223 y:228
			OperatableStateMachine.add('SearchObject',
										hsr_SearchObjectState(search_point='searching_point_0', search_place_type='floor', service_search_floor='/search_object/search_floor', service_update_threshold='/ork_tf_broadcaster/update_threshold', centroid_x_max=2.0, centroid_y_max=1.0, centroid_y_min=-1.0, centroid_z_max=0.3, centroid_z_min=0.0, sleep_time=3.0),
										transitions={'found': 'FetchObject', 'notfound': 'finished', 'failed': 'failed'},
										autonomy={'found': Autonomy.Off, 'notfound': Autonomy.Off, 'failed': Autonomy.Off})

			# x:724 y:468
			OperatableStateMachine.add('SetPoseRecoveryPoint',
										hsr_SetBasePoseByTfNameState(tf_name='recovery_point', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToRecoveryPoint'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:787 y:579
			OperatableStateMachine.add('MoveToRecoveryPoint',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'HSR sweep test', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:855 y:783
			OperatableStateMachine.add('HSR sweep test',
										self.use_behavior(HSRsweeptestSM, 'HSR sweep test'),
										transitions={'finished': 'SetPosePutPoint', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:330 y:298
			OperatableStateMachine.add('Analyse',
										hsr_AnalyseCommandState(default_location='toyshelf', service_name='/wrs_semantics/tidyup_locationOfObject_stge1'),
										transitions={'succeeded': 'FetchObject', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'command': 'object_name', 'object_name': 'object_name', 'location_name': 'location_name', 'location_to_put': 'location_to_put'})

			# x:269 y:475
			OperatableStateMachine.add('SetPosePutPoint',
										hsr_SetBasePoseByTfNameDynState(service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToPuttingPoint'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tf_name': 'location_name', 'pose': 'pose'})

			# x:257 y:581
			OperatableStateMachine.add('PutDyn',
										hsr_PutObjectDynState(put_place_type='shelf', service_name='/grasp/put'),
										transitions={'succeeded': 'MoveToNeutral2', 'failed': 'MoveToNeutral3'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_name': 'location_to_put'})

			# x:335 y:707
			OperatableStateMachine.add('MoveToNeutral3',
										hsr_MoveToNeutralState(),
										transitions={'succeeded': 'SetPosePutPoint', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:204 y:390
			OperatableStateMachine.add('FetchObject',
										hsr_FetchObjectState(fetch_place_type='floor', grasp_srv_name='/grasp/service', stop_tf_srv_name='/ork_tf_broadcaster/stop_publish', target_name='closest'),
										transitions={'succeeded': 'SetPosePutPoint', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
