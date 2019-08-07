#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_set_base_pose_by_tf_name_state import hsr_SetBasePoseByTfNameState
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
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed July 30 2019
@author: ShigemichiMatsuzaki
'''
class HSRTidyUpHereTask1TidyUpSM(Behavior):
	'''
	Behavior of actual tidy up task in Tidy Up Here Task 1 (Tidying up clattered objects on the floor, the coffee table, and the side table)
	'''


	def __init__(self):
		super(HSRTidyUpHereTask1TidyUpSM, self).__init__()
		self.name = 'HSR Tidy Up Here Task 1 Tidy Up'

		# parameters of this behavior
		self.add_parameter('time_limit_task1', 15.0)

		# references to used behaviors
		self.add_behavior(HSRsweeptestSM, 'HSR sweep test')
		self.add_behavior(HSRFetchObjectSM, 'HSR FetchObject1')
		self.add_behavior(HSRMoveSM, 'HSR Move')
		self.add_behavior(HSRFetchObjectSM, 'HSR FetchObject')
		self.add_behavior(HSRMoveSM, 'HSR Move_2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1262 y:668, x:1376 y:171
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['start_time'])
		_state_machine.userdata.start_time = 5.0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:563 y:27
			OperatableStateMachine.add('SetPoseSearchingPoint',
										hsr_SetBasePoseByTfNameState(tf_name='searching_point_floor', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToSearchingPoint'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:1152 y:69
			OperatableStateMachine.add('SpeakErrorMoveBase',
										hsr_SpeakState(sentence='Failed to move', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:755 y:35
			OperatableStateMachine.add('MoveToSearchingPoint',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'CheckElapsedTime1', 'failed': 'SpeakErrorMoveBase'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:1247 y:348
			OperatableStateMachine.add('MoveToNeutral2',
										hsr_MoveToNeutralState(open_hand=False),
										transitions={'succeeded': 'SetPoseSearchingPoint', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:763 y:267
			OperatableStateMachine.add('PutDyn',
										hsr_PutObjectDynState(put_place_type='shelf', service_name='/grasp/put'),
										transitions={'succeeded': 'MoveToNeutral2', 'failed': 'MoveToNeutral3'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_name': 'location_to_put'})

			# x:951 y:386
			OperatableStateMachine.add('MoveToNeutral3',
										hsr_MoveToNeutralState(open_hand=False),
										transitions={'succeeded': 'HSR Move', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:987 y:20
			OperatableStateMachine.add('CheckElapsedTime1',
										hsr_CheckElapsedTimeState(time_limit=self.time_limit_task1),
										transitions={'time_remains': 'FetchObjectInterface1', 'time_up': 'SpeakTimeUp'},
										autonomy={'time_remains': Autonomy.Off, 'time_up': Autonomy.Off},
										remapping={'start_time': 'start_time', 'offset': 'offset'})

			# x:1271 y:464
			OperatableStateMachine.add('SpeakTimeUp',
										hsr_SpeakState(sentence='Time is up. Move on to the next task', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'MoveToNeutralTimeUp'},
										autonomy={'done': Autonomy.Off})

			# x:538 y:268
			OperatableStateMachine.add('CheckElapsedTime2',
										hsr_CheckElapsedTimeState(time_limit=self.time_limit_task1),
										transitions={'time_remains': 'PutDyn', 'time_up': 'SpeakTimeUp'},
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

			# x:340 y:160
			OperatableStateMachine.add('HSR FetchObject1',
										self.use_behavior(HSRFetchObjectSM, 'HSR FetchObject1'),
										transitions={'finished': 'SpeakObjectName', 'grasp_failed': 'SetPoseSearchingPoint', 'not_found': 'SpeakMoveOnToCoffeetable', 'failed': 'FetchObjectInterface1'},
										autonomy={'finished': Autonomy.Inherit, 'grasp_failed': Autonomy.Inherit, 'not_found': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'search_centroid_y_max': 'search_centroid_y_max', 'search_centroid_y_min': 'search_centroid_y_min', 'search_centroid_z_max': 'search_centroid_z_max', 'search_centroid_z_min': 'search_centroid_z_min', 'search_sleep_time': 'search_sleep_time', 'search_is_floor': 'search_is_floor', 'search_centroid_x_max': 'search_centroid_x_max', 'object_name': 'object_name', 'location_name': 'location_name', 'location_to_put': 'location_to_put'})

			# x:324 y:258
			OperatableStateMachine.add('HSR Move',
										self.use_behavior(HSRMoveSM, 'HSR Move'),
										transitions={'succeeded': 'CheckElapsedTime2', 'failed': 'SpeakBeforeSweep'},
										autonomy={'succeeded': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'tf_name': 'location_name'})

			# x:15 y:538
			OperatableStateMachine.add('SetPoseSearchingPointCoffeeTable',
										hsr_SetBasePoseByTfNameState(tf_name='longtableb', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToSearchingPointCoffeeTable'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:258 y:497
			OperatableStateMachine.add('MoveToSearchingPointCoffeeTable',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'CheckElapsedTime3', 'failed': 'MoveToSearchingPointCoffeeTable'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:760 y:559
			OperatableStateMachine.add('HSR FetchObject',
										self.use_behavior(HSRFetchObjectSM, 'HSR FetchObject'),
										transitions={'finished': 'SpeakObjectName2', 'grasp_failed': 'SetPoseSearchingPointCoffeeTable', 'not_found': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'grasp_failed': Autonomy.Inherit, 'not_found': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'search_centroid_y_max': 'search_centroid_y_max', 'search_centroid_y_min': 'search_centroid_y_min', 'search_centroid_z_max': 'search_centroid_z_max', 'search_centroid_z_min': 'search_centroid_z_min', 'search_sleep_time': 'search_sleep_time', 'search_is_floor': 'search_is_floor', 'search_centroid_x_max': 'search_centroid_x_max', 'object_name': 'object_name', 'location_name': 'location_name', 'location_to_put': 'location_to_put'})

			# x:764 y:488
			OperatableStateMachine.add('FetchObjectInterface2',
										hsr_FetchObjectInterfaceState(centroid_x_max=1.0, centroid_y_max=0.6, centroid_y_min=-0.6, centroid_z_max=0.6, centroid_z_min=0.42, sleep_time=5.0, is_floor=False),
										transitions={'done': 'HSR FetchObject'},
										autonomy={'done': Autonomy.Off},
										remapping={'centroid_x_max': 'search_centroid_x_max', 'centroid_y_max': 'search_centroid_y_max', 'centroid_y_min': 'search_centroid_y_min', 'centroid_z_max': 'search_centroid_z_max', 'centroid_z_min': 'search_centroid_z_min', 'sleep_time': 'search_sleep_time', 'is_floor': 'search_is_floor'})

			# x:940 y:679
			OperatableStateMachine.add('SpeakObjectName2',
										hsr_SpeakDynState(sentence="It's +", sentence_when_empty="I couldn't recognize it", topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'HSR Move_2', 'empty': 'HSR Move_2'},
										autonomy={'done': Autonomy.Off, 'empty': Autonomy.Off},
										remapping={'variable': 'object_name'})

			# x:80 y:703
			OperatableStateMachine.add('HSR Move_2',
										self.use_behavior(HSRMoveSM, 'HSR Move_2'),
										transitions={'succeeded': 'Put2', 'failed': 'HSR Move_2'},
										autonomy={'succeeded': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'tf_name': 'location_name'})

			# x:480 y:740
			OperatableStateMachine.add('Put2',
										hsr_PutObjectDynState(put_place_type='shelf', service_name='/grasp/put'),
										transitions={'succeeded': 'SetPoseSearchingPointCoffeeTable', 'failed': 'Put2'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_name': 'location_to_put'})

			# x:47 y:404
			OperatableStateMachine.add('SpeakMoveOnToCoffeetable',
										hsr_SpeakState(sentence='Im moving on to the coffee table', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'SetPoseSearchingPointCoffeeTable'},
										autonomy={'done': Autonomy.Off})

			# x:402 y:395
			OperatableStateMachine.add('SpeakBeforeSweep',
										hsr_SpeakState(sentence='Im sweeping the obstacle', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'HSR sweep test'},
										autonomy={'done': Autonomy.Off})

			# x:555 y:487
			OperatableStateMachine.add('CheckElapsedTime3',
										hsr_CheckElapsedTimeState(time_limit=self.time_limit_task1),
										transitions={'time_remains': 'FetchObjectInterface2', 'time_up': 'SpeakTimeUp'},
										autonomy={'time_remains': Autonomy.Off, 'time_up': Autonomy.Off},
										remapping={'start_time': 'start_time', 'offset': 'offset'})

			# x:1197 y:557
			OperatableStateMachine.add('MoveToNeutralTimeUp',
										hsr_MoveToNeutralState(open_hand=True),
										transitions={'succeeded': 'finished', 'failed': 'finished'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
