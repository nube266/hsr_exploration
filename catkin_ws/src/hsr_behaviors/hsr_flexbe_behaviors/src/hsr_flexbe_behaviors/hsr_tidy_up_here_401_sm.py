#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_start_state import hsr_SetStartTimeState
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBaseState
from hsr_flexbe_states.hsr_set_base_pose_by_tf_name_state import hsr_SetBasePoseByTfNameState
from hsr_flexbe_states.hsr_speak_state import hsr_SpeakState
from hsr_flexbe_states.hsr_move_to_neutral_state import hsr_MoveToNeutralState
from hsr_flexbe_states.hsr_put_object_dyn_state import hsr_PutObjectDynState
from hsr_flexbe_states.hsr_fetch_object_state import hsr_FetchObjectState
from hsr_flexbe_states.hsr_analyse_command_state import hsr_AnalyseCommandState
from hsr_flexbe_states.hsr_set_base_pose_by_tf_name_dyn_state import hsr_SetBasePoseByTfNameDynState
from hsr_flexbe_states.hsr_check_elapsed_time_state import hsr_CheckElapsedTimeState
from hsr_flexbe_states.hsr_search_object_state import hsr_SearchObjectState
from hsr_flexbe_behaviors.hsr_sweep_test_sm import HSRsweeptestSM
from hsr_flexbe_states.hsr_speak_dyn_state import hsr_SpeakDynState
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
		self.add_parameter('time_limit', 5.0)

		# references to used behaviors
		self.add_behavior(HSRsweeptestSM, 'HSR sweep test')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:60 y:551, x:1144 y:241
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:57 y:23
			OperatableStateMachine.add('Start',
										hsr_SetStartTimeState(),
										transitions={'succeeded': 'Speak'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'start_time': 'start_time'})

			# x:560 y:535
			OperatableStateMachine.add('MoveToPuttingPoint',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'CheckElapsedTime2', 'failed': 'HSR sweep test'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:371 y:19
			OperatableStateMachine.add('SetPoseSearchingPoint',
										hsr_SetBasePoseByTfNameState(tf_name='searching_point_0', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToSearchingPoint'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:877 y:147
			OperatableStateMachine.add('SpeakErrorMoveBase',
										hsr_SpeakState(sentence='Failed to move', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:739 y:27
			OperatableStateMachine.add('MoveToSearchingPoint',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'CheckElapsedTime1', 'failed': 'SpeakErrorMoveBase'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:62 y:118
			OperatableStateMachine.add('MoveToNeutral2',
										hsr_MoveToNeutralState(),
										transitions={'succeeded': 'SetPoseSearchingPoint', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:189 y:439
			OperatableStateMachine.add('PutDyn',
										hsr_PutObjectDynState(put_place_type='shelf', service_name='/grasp/put'),
										transitions={'succeeded': 'MoveToNeutral2', 'failed': 'MoveToNeutral3'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_name': 'location_to_put'})

			# x:194 y:704
			OperatableStateMachine.add('MoveToNeutral3',
										hsr_MoveToNeutralState(),
										transitions={'succeeded': 'SetPosePutPoint', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:295 y:288
			OperatableStateMachine.add('FetchObject',
										hsr_FetchObjectState(fetch_place_type='floor', grasp_srv_name='/grasp/service', stop_tf_srv_name='/ork_tf_broadcaster/stop_publish', target_name='closest'),
										transitions={'succeeded': 'Analyse', 'failed': 'MoveToNeutral2'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:523 y:289
			OperatableStateMachine.add('Analyse',
										hsr_AnalyseCommandState(default_location='toyshelf', service_name='/wrs_semantics/tidyup_locationOfObject_stge1'),
										transitions={'succeeded': 'SpeakObjectName', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'command': 'object_name', 'object_name': 'object_name', 'location_name': 'location_name', 'location_to_put': 'location_to_put'})

			# x:422 y:453
			OperatableStateMachine.add('SetPosePutPoint',
										hsr_SetBasePoseByTfNameDynState(service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToPuttingPoint'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tf_name': 'location_name', 'pose': 'pose'})

			# x:472 y:109
			OperatableStateMachine.add('CheckElapsedTime1',
										hsr_CheckElapsedTimeState(time_limit=self.time_limit),
										transitions={'time_remains': 'SearchObject', 'time_up': 'SpeakTimeUp'},
										autonomy={'time_remains': Autonomy.Off, 'time_up': Autonomy.Off},
										remapping={'start_time': 'start_time', 'offset': 'offset'})

			# x:581 y:217
			OperatableStateMachine.add('SpeakTimeUp',
										hsr_SpeakState(sentence='Time is up. Move on to the next task', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:241 y:18
			OperatableStateMachine.add('Speak',
										hsr_SpeakState(sentence='I am going to tidy up here', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'SetPoseSearchingPoint'},
										autonomy={'done': Autonomy.Off})

			# x:366 y:531
			OperatableStateMachine.add('CheckElapsedTime2',
										hsr_CheckElapsedTimeState(time_limit=self.time_limit),
										transitions={'time_remains': 'PutDyn', 'time_up': 'SpeakTimeUp'},
										autonomy={'time_remains': Autonomy.Off, 'time_up': Autonomy.Off},
										remapping={'start_time': 'start_time', 'offset': 'offset'})

			# x:299 y:199
			OperatableStateMachine.add('SearchObject',
										hsr_SearchObjectState(search_point='searching_point_0', search_place_type='floor', service_search_floor='/search_object/search_floor', service_update_threshold='/ork_tf_broadcaster/update_threshold', service_publish_tf='/ork_tf_broadcaster/start_publish', service_stop_tf='/ork_tf_broadcaster/stop_publish', centroid_x_max=1.5, centroid_y_max=1.0, centroid_y_min=-1.0, centroid_z_max=0.2, centroid_z_min=0.0, sleep_time=5.0, is_floor=False),
										transitions={'found': 'FetchObject', 'notfound': 'finished', 'failed': 'failed'},
										autonomy={'found': Autonomy.Off, 'notfound': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_name': 'object_name'})

			# x:721 y:609
			OperatableStateMachine.add('HSR sweep test',
										self.use_behavior(HSRsweeptestSM, 'HSR sweep test'),
										transitions={'finished': 'SetPosePutPoint', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:418 y:378
			OperatableStateMachine.add('SpeakObjectName',
										hsr_SpeakDynState(sentence="It's +", sentence_when_empty="I couldn't recognizeit", topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'SetPosePutPoint'},
										autonomy={'done': Autonomy.Off},
										remapping={'variable': 'object_name'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
