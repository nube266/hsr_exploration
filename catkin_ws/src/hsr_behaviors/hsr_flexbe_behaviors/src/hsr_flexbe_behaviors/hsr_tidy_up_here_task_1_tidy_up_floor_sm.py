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
from hsr_flexbe_states.hsr_move_to_neutral_state import hsr_MoveToNeutralState
from hsr_flexbe_states.hsr_put_object_dyn_state import hsr_PutObjectDynState
from hsr_flexbe_states.hsr_check_elapsed_time_state import hsr_CheckElapsedTimeState
from hsr_flexbe_behaviors.hsr_sweep_test_sm import HSRsweeptestSM
from hsr_flexbe_states.hsr_speak_dyn_state import hsr_SpeakDynState
from hsr_flexbe_states.hsr_fetch_object_interface_state import hsr_FetchObjectInterfaceState
from hsr_flexbe_behaviors.hsr_fetchobject_sm import HSRFetchObjectSM
from hsr_flexbe_behaviors.hsr_move_sm import HSRMoveSM
from hsr_flexbe_states.hsr_set_base_pose_by_tf_name_state import hsr_SetBasePoseByTfNameState
from hsr_flexbe_states.hsr_escape_by_twist_state import hsr_EscapeByTwistState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Aug 1 2019
@author: ShigemichiMatsuzaki
'''
class HSRTidyUpHereTask1TidyUpFloorSM(Behavior):
	'''
	Behavior of actual tidy up task of the floor in Tidy Up Here Task 1 (Tidying up clattered objects on the floor, the coffee table, and the side table)
	'''


	def __init__(self):
		super(HSRTidyUpHereTask1TidyUpFloorSM, self).__init__()
		self.name = 'HSR Tidy Up Here Task 1 Tidy Up Floor'

		# parameters of this behavior
		self.add_parameter('time_limit_task1', 15)

		# references to used behaviors
		self.add_behavior(HSRsweeptestSM, 'HSR sweep test')
		self.add_behavior(HSRFetchObjectSM, 'HSR FetchObject1')
		self.add_behavior(HSRMoveSM, 'HSR Move')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1531 y:322, x:1376 y:171, x:1493 y:216
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'time_up'], input_keys=['start_time'])
		_state_machine.userdata.start_time = 5.0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:141 y:29
			OperatableStateMachine.add('SpeakStart',
										hsr_SpeakState(sentence="I'm going to tidy up the floor first", topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'CheckElapsedTimeBeforeFetch'},
										autonomy={'done': Autonomy.Off})

			# x:1152 y:69
			OperatableStateMachine.add('SpeakErrorMoveBase',
										hsr_SpeakState(sentence='Failed to move', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:957 y:30
			OperatableStateMachine.add('MoveToSearchingPoint',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'FetchObjectInterface1', 'failed': 'SpeakErrorMoveBase'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:1257 y:355
			OperatableStateMachine.add('MoveToNeutralPutSucceed',
										hsr_MoveToNeutralState(open_hand=False),
										transitions={'succeeded': 'CheckElapsedTimeBeforeFetch', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:763 y:267
			OperatableStateMachine.add('PutDyn',
										hsr_PutObjectDynState(put_place_type='shelf', service_name='/grasp/put'),
										transitions={'succeeded': 'MoveToNeutralPutSucceed', 'failed': 'MoveToNeutralPutFail'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_name': 'location_to_put'})

			# x:931 y:430
			OperatableStateMachine.add('MoveToNeutralPutFail',
										hsr_MoveToNeutralState(open_hand=False),
										transitions={'succeeded': 'SendTwist', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:334 y:21
			OperatableStateMachine.add('CheckElapsedTimeBeforeFetch',
										hsr_CheckElapsedTimeState(time_limit=self.time_limit_task1),
										transitions={'time_remains': 'SetPoseSearchingPoint', 'time_up': 'time_up'},
										autonomy={'time_remains': Autonomy.Off, 'time_up': Autonomy.Off},
										remapping={'start_time': 'start_time', 'offset': 'offset'})

			# x:519 y:270
			OperatableStateMachine.add('CheckElapsedTimePut',
										hsr_CheckElapsedTimeState(time_limit=self.time_limit_task1),
										transitions={'time_remains': 'PutDyn', 'time_up': 'MoveToNeutralPutTimeUp'},
										autonomy={'time_remains': Autonomy.Off, 'time_up': Autonomy.Off},
										remapping={'start_time': 'start_time', 'offset': 'offset'})

			# x:636 y:379
			OperatableStateMachine.add('HSR sweep test',
										self.use_behavior(HSRsweeptestSM, 'HSR sweep test'),
										transitions={'finished': 'HSR Move', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:765 y:144
			OperatableStateMachine.add('SpeakObjectName',
										hsr_SpeakDynState(sentence="It's +", sentence_when_empty="I couldn't recognize it", topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'HSR Move', 'empty': 'HSR Move'},
										autonomy={'done': Autonomy.Off, 'empty': Autonomy.Off},
										remapping={'variable': 'object_name'})

			# x:22 y:150
			OperatableStateMachine.add('FetchObjectInterface1',
										hsr_FetchObjectInterfaceState(centroid_x_max=1.3, centroid_y_max=1.0, centroid_y_min=-1.0, centroid_z_max=0.2, centroid_z_min=0.0, sleep_time=5.0, is_floor=False),
										transitions={'done': 'HSR FetchObject1'},
										autonomy={'done': Autonomy.Off},
										remapping={'centroid_x_max': 'search_centroid_x_max', 'centroid_y_max': 'search_centroid_y_max', 'centroid_y_min': 'search_centroid_y_min', 'centroid_z_max': 'search_centroid_z_max', 'centroid_z_min': 'search_centroid_z_min', 'sleep_time': 'search_sleep_time', 'is_floor': 'search_is_floor'})

			# x:367 y:187
			OperatableStateMachine.add('HSR FetchObject1',
										self.use_behavior(HSRFetchObjectSM, 'HSR FetchObject1'),
										transitions={'finished': 'SpeakObjectName', 'grasp_failed': 'SetPoseSearchingPoint', 'not_found': 'finished', 'failed': 'FetchObjectInterface1'},
										autonomy={'finished': Autonomy.Inherit, 'grasp_failed': Autonomy.Inherit, 'not_found': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'search_centroid_y_max': 'search_centroid_y_max', 'search_centroid_y_min': 'search_centroid_y_min', 'search_centroid_z_max': 'search_centroid_z_max', 'search_centroid_z_min': 'search_centroid_z_min', 'search_sleep_time': 'search_sleep_time', 'search_is_floor': 'search_is_floor', 'search_centroid_x_max': 'search_centroid_x_max', 'object_name': 'object_name', 'location_name': 'location_name', 'location_to_put': 'location_to_put'})

			# x:253 y:270
			OperatableStateMachine.add('HSR Move',
										self.use_behavior(HSRMoveSM, 'HSR Move'),
										transitions={'succeeded': 'CheckElapsedTimePut', 'failed': 'SpeakBeforeSweep'},
										autonomy={'succeeded': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'tf_name': 'location_name'})

			# x:402 y:395
			OperatableStateMachine.add('SpeakBeforeSweep',
										hsr_SpeakState(sentence='Im sweeping the obstacle', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'HSR sweep test'},
										autonomy={'done': Autonomy.Off})

			# x:649 y:21
			OperatableStateMachine.add('SetPoseSearchingPoint',
										hsr_SetBasePoseByTfNameState(tf_name='searching_point_floor', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToSearchingPoint'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:764 y:483
			OperatableStateMachine.add('SendTwist',
										hsr_EscapeByTwistState(topic='/hsrb/command_velocity', linear_x=0.0, linear_y=0.0, angular=0.3, duration=1.0),
										transitions={'completed': 'HSR Move'},
										autonomy={'completed': Autonomy.Off})

			# x:1169 y:483
			OperatableStateMachine.add('MoveToNeutralPutTimeUp',
										hsr_MoveToNeutralState(open_hand=True),
										transitions={'succeeded': 'time_up', 'failed': 'time_up'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
