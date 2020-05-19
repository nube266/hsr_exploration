#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.wait_state import WaitState
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBaseState
from hsr_flexbe_states.hsr_set_base_pose_by_tf_name_state import hsr_SetBasePoseByTfNameState
from hsr_flexbe_states.hsr_put_object_state import hsr_PutObjectState
from hsr_flexbe_states.hsr_escape_by_twist_state import hsr_EscapeByTwistState
from hsr_flexbe_states.hsr_speak_state import hsr_SpeakState
from hsr_flexbe_states.hsr_fetch_object_interface_state import hsr_FetchObjectInterfaceState
from hsr_flexbe_behaviors.hsr_fetchobjectnew_sm import HSRFetchObjectNewSM
from hsr_flexbe_states.hsr_open_drawer_state import hsr_OpenDrawerState
from hsr_flexbe_states.hsr_wait_press_wrist_state import hsr_WaitWristPressedState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Aug 20 2019
@author: ShigemichiMatsuzaki
'''
class HSROpenCampus2019backupSM(Behavior):
	'''
	Demo of Tidy Up Here in Open Campus 2019
	'''


	def __init__(self):
		super(HSROpenCampus2019backupSM, self).__init__()
		self.name = 'HSR Open Campus 2019 backup'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(HSRFetchObjectNewSM, 'HSR FetchObjectShortTable')
		self.add_behavior(HSRFetchObjectNewSM, 'HSR FetchObjectFloor')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:181 y:585, x:715 y:201
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('WaitBeforeWrist',
										WaitState(wait_time=1.0),
										transitions={'done': 'SpeakBeforeWrist'},
										autonomy={'done': Autonomy.Off})

			# x:1109 y:138
			OperatableStateMachine.add('MoveToFloor',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'SetFetchFloorArea', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:1080 y:497
			OperatableStateMachine.add('SetPoseDrawerToPut',
										hsr_SetBasePoseByTfNameState(tf_name='drawer', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToDrawerToPut'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:1110 y:593
			OperatableStateMachine.add('MoveToDrawerToPut',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'SpeakPut1', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:1537 y:598
			OperatableStateMachine.add('Put',
										hsr_PutObjectState(put_place_type='shelf', target_name='drawer_0', service_name='/grasp/put'),
										transitions={'succeeded': 'SpeakPutSucceeded', 'failed': 'Escape'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1398 y:500
			OperatableStateMachine.add('Escape',
										hsr_EscapeByTwistState(topic='/hsrb/command_velocity', linear_x=0.0, linear_y=0.0, angular=0.3, duration=1.0),
										transitions={'completed': 'SetPoseDrawerToPut'},
										autonomy={'completed': Autonomy.Off})

			# x:311 y:111
			OperatableStateMachine.add('SpeakDrawerStart',
										hsr_SpeakState(sentence='片付けを始めるよ', topic='/talk_request', interrupting=False, queueing=False, language=0),
										transitions={'done': 'SetPoseDrawer'},
										autonomy={'done': Autonomy.Off})

			# x:320 y:208
			OperatableStateMachine.add('WaitStart',
										WaitState(wait_time=0.1),
										transitions={'done': 'SpeakDrawerStart'},
										autonomy={'done': Autonomy.Off})

			# x:1355 y:153
			OperatableStateMachine.add('SpeakGraspFailed',
										hsr_SpeakState(sentence='拾うのに失敗したからやり直すよ', topic='/talk_request', interrupting=False, queueing=False, language=0),
										transitions={'done': 'SetPoseFloor'},
										autonomy={'done': Autonomy.Off})

			# x:1521 y:26
			OperatableStateMachine.add('SpeakPutSucceeded',
										hsr_SpeakState(sentence='もう一度探すよ', topic='/talk_request', interrupting=False, queueing=False, language=0),
										transitions={'done': 'SetPoseFloor'},
										autonomy={'done': Autonomy.Off})

			# x:867 y:701
			OperatableStateMachine.add('SpeakFinishFloor',
										hsr_SpeakState(sentence='次はテーブルを見るよ', topic='/talk_request', interrupting=False, queueing=False, language=0),
										transitions={'done': 'SetPoseShortTable'},
										autonomy={'done': Autonomy.Off})

			# x:1087 y:255
			OperatableStateMachine.add('SetFetchFloorArea',
										hsr_FetchObjectInterfaceState(centroid_x_max=1.7, centroid_y_max=1.0, centroid_y_min=-1.0, centroid_z_max=0.2, centroid_z_min=0.0, sleep_time=5.0, is_floor=False),
										transitions={'done': 'HSR FetchObjectFloor'},
										autonomy={'done': Autonomy.Off},
										remapping={'centroid_x_max': 'search_centroid_x_max', 'centroid_y_max': 'search_centroid_y_max', 'centroid_y_min': 'search_centroid_y_min', 'centroid_z_max': 'search_centroid_z_max', 'centroid_z_min': 'search_centroid_z_min', 'sleep_time': 'search_sleep_time', 'is_floor': 'search_is_floor'})

			# x:598 y:704
			OperatableStateMachine.add('SetPoseShortTable',
										hsr_SetBasePoseByTfNameState(tf_name='shorttable', service_name='/pose_server/getPose'),
										transitions={'completed': 'MveToShortTable'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:624 y:577
			OperatableStateMachine.add('MveToShortTable',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'SetFetchTableArea', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:383 y:583
			OperatableStateMachine.add('SetFetchTableArea',
										hsr_FetchObjectInterfaceState(centroid_x_max=1.2, centroid_y_max=0.4, centroid_y_min=-0.38, centroid_z_max=0.6, centroid_z_min=0.38, sleep_time=5.0, is_floor=False),
										transitions={'done': 'HSR FetchObjectShortTable'},
										autonomy={'done': Autonomy.Off},
										remapping={'centroid_x_max': 'search_centroid_x_max', 'centroid_y_max': 'search_centroid_y_max', 'centroid_y_min': 'search_centroid_y_min', 'centroid_z_max': 'search_centroid_z_max', 'centroid_z_min': 'search_centroid_z_min', 'sleep_time': 'search_sleep_time', 'is_floor': 'search_is_floor'})

			# x:386 y:350
			OperatableStateMachine.add('SetPoseDrawerToPut2',
										hsr_SetBasePoseByTfNameState(tf_name='drawer', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToDrawerToPut2'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:392 y:285
			OperatableStateMachine.add('MoveToDrawerToPut2',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'SpeakPut2', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:57 y:284
			OperatableStateMachine.add('Put2',
										hsr_PutObjectState(put_place_type='shelf', target_name='drawer_0', service_name='/grasp/put'),
										transitions={'succeeded': 'SpeakPutSucceeded2', 'failed': 'Escape2'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:135 y:385
			OperatableStateMachine.add('Escape2',
										hsr_EscapeByTwistState(topic='/hsrb/command_velocity', linear_x=0.0, linear_y=0.0, angular=0.3, duration=1.0),
										transitions={'completed': 'SetPoseDrawerToPut2'},
										autonomy={'completed': Autonomy.Off})

			# x:48 y:702
			OperatableStateMachine.add('SpeakPutSucceeded2',
										hsr_SpeakState(sentence='もう一度探すよ', topic='/talk_request', interrupting=False, queueing=False, language=0),
										transitions={'done': 'SetPoseShortTable'},
										autonomy={'done': Autonomy.Off})

			# x:145 y:476
			OperatableStateMachine.add('SpeakFinishShortTable',
										hsr_SpeakState(sentence='これで終わりだよ', topic='/talk_request', interrupting=False, queueing=False, language=0),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:182 y:636
			OperatableStateMachine.add('SpeakGraspFailed2',
										hsr_SpeakState(sentence='もう一度拾います', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'SetPoseShortTable'},
										autonomy={'done': Autonomy.Off})

			# x:289 y:25
			OperatableStateMachine.add('SetPoseDrawer',
										hsr_SetBasePoseByTfNameState(tf_name='drawer', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToDrawer'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:1346 y:593
			OperatableStateMachine.add('SpeakPut1',
										hsr_SpeakState(sentence='引き出しに入れるよ', topic='/talk_request', interrupting=False, queueing=False, language=0),
										transitions={'done': 'Put'},
										autonomy={'done': Autonomy.Off})

			# x:221 y:280
			OperatableStateMachine.add('SpeakPut2',
										hsr_SpeakState(sentence='引き出しに入れるよ', topic='/talk_request', interrupting=False, queueing=False, language=0),
										transitions={'done': 'Put2'},
										autonomy={'done': Autonomy.Off})

			# x:1110 y:423
			OperatableStateMachine.add('SpeakSuccessFetch',
										hsr_SpeakState(sentence='大成功', topic='/talk_request', interrupting=False, queueing=False, language=0),
										transitions={'done': 'SetPoseDrawerToPut'},
										autonomy={'done': Autonomy.Off})

			# x:407 y:421
			OperatableStateMachine.add('SpeakSuccessFetch2',
										hsr_SpeakState(sentence='大成功', topic='/talk_request', interrupting=False, queueing=False, language=0),
										transitions={'done': 'SetPoseDrawerToPut2'},
										autonomy={'done': Autonomy.Off})

			# x:373 y:492
			OperatableStateMachine.add('HSR FetchObjectShortTable',
										self.use_behavior(HSRFetchObjectNewSM, 'HSR FetchObjectShortTable'),
										transitions={'finished': 'SpeakSuccessFetch2', 'grasp_failed': 'SpeakGraspFailed2', 'not_found': 'SpeakFinishShortTable', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'grasp_failed': Autonomy.Inherit, 'not_found': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'search_centroid_y_max': 'search_centroid_y_max', 'search_centroid_y_min': 'search_centroid_y_min', 'search_centroid_z_max': 'search_centroid_z_max', 'search_centroid_z_min': 'search_centroid_z_min', 'search_sleep_time': 'search_sleep_time', 'search_is_floor': 'search_is_floor', 'search_centroid_x_max': 'search_centroid_x_max', 'object_name': 'object_name', 'location_name': 'location_name', 'location_to_put': 'location_to_put'})

			# x:1095 y:332
			OperatableStateMachine.add('HSR FetchObjectFloor',
										self.use_behavior(HSRFetchObjectNewSM, 'HSR FetchObjectFloor'),
										transitions={'finished': 'SpeakSuccessFetch', 'grasp_failed': 'SpeakGraspFailed', 'not_found': 'SpeakFinishFloor', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'grasp_failed': Autonomy.Inherit, 'not_found': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'search_centroid_y_max': 'search_centroid_y_max', 'search_centroid_y_min': 'search_centroid_y_min', 'search_centroid_z_max': 'search_centroid_z_max', 'search_centroid_z_min': 'search_centroid_z_min', 'search_sleep_time': 'search_sleep_time', 'search_is_floor': 'search_is_floor', 'search_centroid_x_max': 'search_centroid_x_max', 'object_name': 'object_name', 'location_name': 'location_name', 'location_to_put': 'location_to_put'})

			# x:610 y:34
			OperatableStateMachine.add('MoveToDrawer',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'OpenDrawer', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:865 y:48
			OperatableStateMachine.add('OpenDrawer',
										hsr_OpenDrawerState(open_srv_name='/grasp/open_drawer', height=0),
										transitions={'succeeded': 'SetPoseFloor', 'failed': 'SetPoseFloor'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1075 y:26
			OperatableStateMachine.add('SetPoseFloor',
										hsr_SetBasePoseByTfNameState(tf_name='floor', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToFloor'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:38 y:194
			OperatableStateMachine.add('WaitWristPressed',
										hsr_WaitWristPressedState(),
										transitions={'succeeded': 'WaitStart'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'start_time': 'start_time'})

			# x:30 y:102
			OperatableStateMachine.add('SpeakBeforeWrist',
										hsr_SpeakState(sentence='Press my wrist', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'WaitWristPressed'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
