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
from hsr_flexbe_states.hsr_set_base_pose_by_tf_name_state import hsr_SetBasePoseByTfNameState
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBaseState
from hsr_flexbe_behaviors.hsr_fetchobject_sm import HSRFetchObjectSM
from hsr_flexbe_states.hsr_fetch_object_interface_state import hsr_FetchObjectInterfaceState
from hsr_flexbe_states.hsr_speak_dyn_state import hsr_SpeakDynState
from hsr_flexbe_behaviors.hsr_move_sm import HSRMoveSM
from hsr_flexbe_states.hsr_put_object_dyn_state import hsr_PutObjectDynState
from hsr_flexbe_states.hsr_check_elapsed_time_state import hsr_CheckElapsedTimeState
from hsr_flexbe_states.hsr_move_to_neutral_state import hsr_MoveToNeutralState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Aug 1 2019
@author: ShigemichiMatsuzaki
'''
class HSRTidyUpHereTask1TidyUpTallTableSM(Behavior):
	'''
	Behavior of actual tidy up task of the tall table in Tidy Up Here Task 1 (Tidying up clattered objects on the floor, the coffee table, and the side table)
	'''


	def __init__(self):
		super(HSRTidyUpHereTask1TidyUpTallTableSM, self).__init__()
		self.name = 'HSR Tidy Up Here Task 1 Tidy Up Tall Table'

		# parameters of this behavior
		self.add_parameter('time_limit_task1', 15.0)

		# references to used behaviors
		self.add_behavior(HSRFetchObjectSM, 'HSR FetchObject')
		self.add_behavior(HSRMoveSM, 'HSR Move_2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1397 y:315, x:847 y:410, x:1404 y:206
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'time_up'], input_keys=['start_time'])
		_state_machine.userdata.start_time = 5.0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:45 y:36
			OperatableStateMachine.add('SpeakMoveOnToTallTable',
										hsr_SpeakState(sentence='Im moving on to the tall table', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'SetPoseSearchingPointCoffeeTable'},
										autonomy={'done': Autonomy.Off})

			# x:231 y:39
			OperatableStateMachine.add('SetPoseSearchingPointCoffeeTable',
										hsr_SetBasePoseByTfNameState(tf_name='talltable', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToSearchingPointCoffeeTable'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:487 y:38
			OperatableStateMachine.add('MoveToSearchingPointCoffeeTable',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'CheckElapsedTimeBeforeFetch', 'failed': 'MoveToSearchingPointCoffeeTable'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:368 y:183
			OperatableStateMachine.add('HSR FetchObject',
										self.use_behavior(HSRFetchObjectSM, 'HSR FetchObject'),
										transitions={'finished': 'SpeakObjectName2', 'grasp_failed': 'SetPoseSearchingPointCoffeeTable', 'not_found': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'grasp_failed': Autonomy.Inherit, 'not_found': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'search_centroid_y_max': 'search_centroid_y_max', 'search_centroid_y_min': 'search_centroid_y_min', 'search_centroid_z_max': 'search_centroid_z_max', 'search_centroid_z_min': 'search_centroid_z_min', 'search_sleep_time': 'search_sleep_time', 'search_is_floor': 'search_is_floor', 'search_centroid_x_max': 'search_centroid_x_max', 'object_name': 'object_name', 'location_name': 'location_name', 'location_to_put': 'location_to_put'})

			# x:45 y:160
			OperatableStateMachine.add('FetchObjectInterface2',
										hsr_FetchObjectInterfaceState(centroid_x_max=1.0, centroid_y_max=0.25, centroid_y_min=-0.25, centroid_z_max=0.80, centroid_z_min=0.61, sleep_time=5.0, is_floor=False),
										transitions={'done': 'HSR FetchObject'},
										autonomy={'done': Autonomy.Off},
										remapping={'centroid_x_max': 'search_centroid_x_max', 'centroid_y_max': 'search_centroid_y_max', 'centroid_y_min': 'search_centroid_y_min', 'centroid_z_max': 'search_centroid_z_max', 'centroid_z_min': 'search_centroid_z_min', 'sleep_time': 'search_sleep_time', 'is_floor': 'search_is_floor'})

			# x:75 y:277
			OperatableStateMachine.add('SpeakObjectName2',
										hsr_SpeakDynState(sentence="It's +", sentence_when_empty="I couldn't recognize it", topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'HSR Move_2', 'empty': 'HSR Move_2'},
										autonomy={'done': Autonomy.Off, 'empty': Autonomy.Off},
										remapping={'variable': 'object_name'})

			# x:310 y:381
			OperatableStateMachine.add('HSR Move_2',
										self.use_behavior(HSRMoveSM, 'HSR Move_2'),
										transitions={'succeeded': 'CheckElapsedTimeBeforePut', 'failed': 'HSR Move_2'},
										autonomy={'succeeded': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'tf_name': 'location_name'})

			# x:637 y:175
			OperatableStateMachine.add('Put2',
										hsr_PutObjectDynState(put_place_type='shelf', service_name='/grasp/put'),
										transitions={'succeeded': 'CheckElapsedTimeBeforeMoveToTable', 'failed': 'CheckElapsedTimeBeforePut'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_name': 'location_to_put'})

			# x:748 y:39
			OperatableStateMachine.add('CheckElapsedTimeBeforeFetch',
										hsr_CheckElapsedTimeState(time_limit=self.time_limit_task1),
										transitions={'time_remains': 'FetchObjectInterface2', 'time_up': 'SpeakTimeUp'},
										autonomy={'time_remains': Autonomy.Off, 'time_up': Autonomy.Off},
										remapping={'start_time': 'start_time', 'offset': 'offset'})

			# x:1187 y:172
			OperatableStateMachine.add('MoveToNeutralTimeUp',
										hsr_MoveToNeutralState(open_hand=True),
										transitions={'succeeded': 'time_up', 'failed': 'time_up'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:900 y:155
			OperatableStateMachine.add('CheckElapsedTimeBeforeMoveToTable',
										hsr_CheckElapsedTimeState(time_limit=self.time_limit_task1),
										transitions={'time_remains': 'SetPoseSearchingPointCoffeeTable', 'time_up': 'SpeakTimeUp'},
										autonomy={'time_remains': Autonomy.Off, 'time_up': Autonomy.Off},
										remapping={'start_time': 'start_time', 'offset': 'offset'})

			# x:478 y:297
			OperatableStateMachine.add('CheckElapsedTimeBeforePut',
										hsr_CheckElapsedTimeState(time_limit=self.time_limit_task1),
										transitions={'time_remains': 'Put2', 'time_up': 'SpeakTimeUp'},
										autonomy={'time_remains': Autonomy.Off, 'time_up': Autonomy.Off},
										remapping={'start_time': 'start_time', 'offset': 'offset'})

			# x:1065 y:62
			OperatableStateMachine.add('SpeakTimeUp',
										hsr_SpeakState(sentence='Time is up. Move on to the next task', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'MoveToNeutralTimeUp'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
