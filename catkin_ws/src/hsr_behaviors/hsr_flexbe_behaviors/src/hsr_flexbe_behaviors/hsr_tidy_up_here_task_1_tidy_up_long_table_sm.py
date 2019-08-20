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
from hsr_flexbe_states.hsr_fetch_object_interface_state import hsr_FetchObjectInterfaceState
from hsr_flexbe_states.hsr_speak_dyn_state import hsr_SpeakDynState
from hsr_flexbe_behaviors.hsr_move_sm import HSRMoveSM
from hsr_flexbe_states.hsr_put_object_dyn_state import hsr_PutObjectDynState
from hsr_flexbe_states.hsr_check_elapsed_time_state import hsr_CheckElapsedTimeState
from hsr_flexbe_states.hsr_move_to_neutral_state import hsr_MoveToNeutralState
from hsr_flexbe_behaviors.hsr_fetchobjectlongtable_sm import HSRFetchObjectLongTableSM
from hsr_flexbe_states.hsr_escape_by_twist_state import hsr_EscapeByTwistState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Aug 1 2019
@author: ShigemichiMatsuzaki
'''
class HSRTidyUpHereTask1TidyUpLongTableSM(Behavior):
	'''
	Behavior of actual tidy up task of the long table in Tidy Up Here Task 1 (Tidying up clattered objects on the floor, the coffee table, and the side table)
	'''


	def __init__(self):
		super(HSRTidyUpHereTask1TidyUpLongTableSM, self).__init__()
		self.name = 'HSR Tidy Up Here Task 1 Tidy Up Long Table'

		# parameters of this behavior
		self.add_parameter('time_limit_task1', 15.0)

		# references to used behaviors
		self.add_behavior(HSRMoveSM, 'HSR Move_2')
		self.add_behavior(HSRFetchObjectLongTableSM, 'HSR FetchObjectLongTable')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1397 y:315, x:768 y:408, x:1486 y:209
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'time_up'], input_keys=['start_time'], output_keys=['offset'])
		_state_machine.userdata.start_time = 5.0
		_state_machine.userdata.offset = 0.0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:45 y:36
			OperatableStateMachine.add('SpeakMoveOnToCoffeetable',
										hsr_SpeakState(sentence='Im moving on to the coffee table', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'CheckElapsedTimeBeforeFetch'},
										autonomy={'done': Autonomy.Off})

			# x:542 y:16
			OperatableStateMachine.add('SetPoseSearchingPointTallTable',
										hsr_SetBasePoseByTfNameState(tf_name='longtableb', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToSearchingPointTallTable'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:928 y:11
			OperatableStateMachine.add('MoveToSearchingPointTallTable',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'FetchObjectInterface2', 'failed': 'MoveToSearchingPointTallTable'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:45 y:160
			OperatableStateMachine.add('FetchObjectInterface2',
										hsr_FetchObjectInterfaceState(centroid_x_max=1.0, centroid_y_max=0.6, centroid_y_min=-0.6, centroid_z_max=0.6, centroid_z_min=0.42, sleep_time=5.0, is_floor=False),
										transitions={'done': 'HSR FetchObjectLongTable'},
										autonomy={'done': Autonomy.Off},
										remapping={'centroid_x_max': 'search_centroid_x_max', 'centroid_y_max': 'search_centroid_y_max', 'centroid_y_min': 'search_centroid_y_min', 'centroid_z_max': 'search_centroid_z_max', 'centroid_z_min': 'search_centroid_z_min', 'sleep_time': 'search_sleep_time', 'is_floor': 'search_is_floor'})

			# x:75 y:277
			OperatableStateMachine.add('SpeakObjectName2',
										hsr_SpeakDynState(sentence="It's +", sentence_when_empty="I couldn't recognize it", topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'HSR Move_2', 'empty': 'HSR Move_2'},
										autonomy={'done': Autonomy.Off, 'empty': Autonomy.Off},
										remapping={'variable': 'object_name'})

			# x:285 y:401
			OperatableStateMachine.add('HSR Move_2',
										self.use_behavior(HSRMoveSM, 'HSR Move_2'),
										transitions={'succeeded': 'CheckElapsedTimeBeforePut', 'failed': 'HSR Move_2'},
										autonomy={'succeeded': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'tf_name': 'location_name'})

			# x:904 y:319
			OperatableStateMachine.add('Put2',
										hsr_PutObjectDynState(put_place_type='shelf', service_name='/grasp/put'),
										transitions={'succeeded': 'CheckElapsedTimeBeforeFetch', 'failed': 'SendTwist'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_name': 'location_to_put'})

			# x:253 y:29
			OperatableStateMachine.add('CheckElapsedTimeBeforeFetch',
										hsr_CheckElapsedTimeState(time_limit=self.time_limit_task1, margin=0.8),
										transitions={'time_remains': 'SetPoseSearchingPointTallTable', 'time_up': 'SpeakTimeUp'},
										autonomy={'time_remains': Autonomy.Off, 'time_up': Autonomy.Off},
										remapping={'start_time': 'start_time', 'offset': 'offset'})

			# x:1211 y:221
			OperatableStateMachine.add('MoveToNeutralTimeUp',
										hsr_MoveToNeutralState(open_hand=True),
										transitions={'succeeded': 'time_up', 'failed': 'time_up'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:468 y:308
			OperatableStateMachine.add('CheckElapsedTimeBeforePut',
										hsr_CheckElapsedTimeState(time_limit=self.time_limit_task1, margin=0.3),
										transitions={'time_remains': 'Put2', 'time_up': 'SpeakTimeUp'},
										autonomy={'time_remains': Autonomy.Off, 'time_up': Autonomy.Off},
										remapping={'start_time': 'start_time', 'offset': 'offset'})

			# x:1174 y:100
			OperatableStateMachine.add('SpeakTimeUp',
										hsr_SpeakState(sentence='Time is up. Move on to the next task', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'MoveToNeutralTimeUp'},
										autonomy={'done': Autonomy.Off})

			# x:438 y:226
			OperatableStateMachine.add('HSR FetchObjectLongTable',
										self.use_behavior(HSRFetchObjectLongTableSM, 'HSR FetchObjectLongTable'),
										transitions={'finished': 'SpeakObjectName2', 'grasp_failed': 'Neutral', 'not_found': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'grasp_failed': Autonomy.Inherit, 'not_found': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'search_centroid_y_max': 'search_centroid_y_max', 'search_centroid_y_min': 'search_centroid_y_min', 'search_centroid_z_max': 'search_centroid_z_max', 'search_centroid_z_min': 'search_centroid_z_min', 'search_sleep_time': 'search_sleep_time', 'search_is_floor': 'search_is_floor', 'search_centroid_x_max': 'search_centroid_x_max', 'object_name': 'object_name', 'location_name': 'location_name', 'location_to_put': 'location_to_put'})

			# x:758 y:514
			OperatableStateMachine.add('SendTwist',
										hsr_EscapeByTwistState(topic='/hsrb/command_velocity', linear_x=0.0, linear_y=0.0, angular=0.3, duration=1.0),
										transitions={'completed': 'HSR Move_2'},
										autonomy={'completed': Autonomy.Off})

			# x:320 y:116
			OperatableStateMachine.add('Neutral',
										hsr_MoveToNeutralState(open_hand=False),
										transitions={'succeeded': 'CheckElapsedTimeBeforeFetch', 'failed': 'CheckElapsedTimeBeforeFetch'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
