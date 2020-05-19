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
from hsr_flexbe_behaviors.hsr_sweep_test_sm import HSRsweeptestSM
from hsr_flexbe_states.hsr_fetch_object_interface_state import hsr_FetchObjectInterfaceState
from hsr_flexbe_states.hsr_set_base_pose_by_tf_name_state import hsr_SetBasePoseByTfNameState
from hsr_flexbe_states.hsr_escape_by_twist_state import hsr_EscapeByTwistState
from hsr_flexbe_behaviors.hsr_fetchobjectnew_sm import HSRFetchObjectNewSM
from hsr_flexbe_states.hsr_put_object_state import hsr_PutObjectState
from hsr_flexbe_states.hsr_wait_press_wrist_state import hsr_WaitWristPressedState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Aug 1 2019
@author: ShigemichiMatsuzaki
'''
class HSROpenCampusTidyUpFloorSM(Behavior):
	'''
	Behavior of actual tidy up task of the floor in Tidy Up Here Task 1 (Tidying up clattered objects on the floor, the coffee table, and the side table)
	'''


	def __init__(self):
		super(HSROpenCampusTidyUpFloorSM, self).__init__()
		self.name = 'HSR Open Campus Tidy Up Floor'

		# parameters of this behavior
		self.add_parameter('time_limit_task1', 15.0)

		# references to used behaviors
		self.add_behavior(HSRsweeptestSM, 'HSR sweep test')
		self.add_behavior(HSRFetchObjectNewSM, 'HSR FetchObjectNew')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1531 y:322, x:1335 y:176, x:1493 y:216
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'time_up'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('SpeakBeforeWrist',
										hsr_SpeakState(sentence="手首を押してね", topic='/talk_request', interrupting=False, queueing=False, language=0),
										transitions={'done': 'PressWrist'},
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
										transitions={'succeeded': 'SetPoseSearchingPoint', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:931 y:430
			OperatableStateMachine.add('MoveToNeutralPutFail',
										hsr_MoveToNeutralState(open_hand=False),
										transitions={'succeeded': 'SendTwist', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:230 y:352
			OperatableStateMachine.add('HSR sweep test',
										self.use_behavior(HSRsweeptestSM, 'HSR sweep test'),
										transitions={'finished': 'SetPoseDrawer', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:22 y:150
			OperatableStateMachine.add('FetchObjectInterface1',
										hsr_FetchObjectInterfaceState(centroid_x_max=1.7, centroid_y_max=1.2, centroid_y_min=-1.2, centroid_z_max=0.2, centroid_z_min=0.0, sleep_time=5.0, is_floor=False),
										transitions={'done': 'HSR FetchObjectNew'},
										autonomy={'done': Autonomy.Off},
										remapping={'centroid_x_max': 'search_centroid_x_max', 'centroid_y_max': 'search_centroid_y_max', 'centroid_y_min': 'search_centroid_y_min', 'centroid_z_max': 'search_centroid_z_max', 'centroid_z_min': 'search_centroid_z_min', 'sleep_time': 'search_sleep_time', 'is_floor': 'search_is_floor'})

			# x:402 y:395
			OperatableStateMachine.add('SpeakBeforeSweep',
										hsr_SpeakState(sentence='Im sweeping the obstacle', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'HSR sweep test'},
										autonomy={'done': Autonomy.Off})

			# x:649 y:21
			OperatableStateMachine.add('SetPoseSearchingPoint',
										hsr_SetBasePoseByTfNameState(tf_name='floor', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToSearchingPoint'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:764 y:483
			OperatableStateMachine.add('SendTwist',
										hsr_EscapeByTwistState(topic='/hsrb/command_velocity', linear_x=0.0, linear_y=0.0, angular=0.3, duration=1.0),
										transitions={'completed': 'SetPoseDrawer'},
										autonomy={'completed': Autonomy.Off})

			# x:59 y:252
			OperatableStateMachine.add('MoveToNeutralGraspFail',
										hsr_MoveToNeutralState(open_hand=False),
										transitions={'succeeded': 'FetchObjectInterface1', 'failed': 'MoveToNeutralGraspFail'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:359 y:156
			OperatableStateMachine.add('HSR FetchObjectNew',
										self.use_behavior(HSRFetchObjectNewSM, 'HSR FetchObjectNew'),
										transitions={'finished': 'SetPoseDrawer', 'grasp_failed': 'SetPoseSearchingPoint', 'not_found': 'finished', 'failed': 'MoveToNeutralGraspFail'},
										autonomy={'finished': Autonomy.Inherit, 'grasp_failed': Autonomy.Inherit, 'not_found': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'search_centroid_y_max': 'search_centroid_y_max', 'search_centroid_y_min': 'search_centroid_y_min', 'search_centroid_z_max': 'search_centroid_z_max', 'search_centroid_z_min': 'search_centroid_z_min', 'search_sleep_time': 'search_sleep_time', 'search_is_floor': 'search_is_floor', 'search_centroid_x_max': 'search_centroid_x_max', 'object_name': 'object_name', 'location_name': 'location_name', 'location_to_put': 'location_to_put'})

			# x:742 y:240
			OperatableStateMachine.add('Put',
										hsr_PutObjectState(put_place_type='shelf', target_name='drawer_0', service_name='/grasp/put'),
										transitions={'succeeded': 'MoveToNeutralPutSucceed', 'failed': 'MoveToNeutralPutFail'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:395 y:265
			OperatableStateMachine.add('SetPoseDrawer',
										hsr_SetBasePoseByTfNameState(tf_name='drawer', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToDrawer'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:598 y:270
			OperatableStateMachine.add('MoveToDrawer',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'Put', 'failed': 'SpeakBeforeSweep'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:222 y:26
			OperatableStateMachine.add('PressWrist',
										hsr_WaitWristPressedState(),
										transitions={'succeeded': 'SpeakStart'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'start_time': 'start_time'})

			# x:395 y:20
			OperatableStateMachine.add('SpeakStart',
										hsr_SpeakState(sentence="片付け、頑張るぞー", topic='/talk_request', interrupting=False, queueing=False, language=0),
										transitions={'done': 'SetPoseSearchingPoint'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
