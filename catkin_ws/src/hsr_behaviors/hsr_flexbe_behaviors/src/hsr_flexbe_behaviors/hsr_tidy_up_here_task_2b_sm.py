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
from flexbe_states.publisher_string_state import PublisherStringState
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBaseState
from hsr_flexbe_states.hsr_speak_state import hsr_SpeakState
from hsr_flexbe_states.hsr_pass_object_state import hsr_PassObjectState
from hsr_flexbe_states.hsr_fetch_object_dyn_state import hsr_FetchObjectDynState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jul 22 2019
@author: Shigemichi Matsuzaki
'''
class HSRTidyUpHereTask2bSM(Behavior):
	'''
	Behavior for Tidy Up Here Task 2b (bringing a designated object from the shelf)
	'''


	def __init__(self):
		super(HSRTidyUpHereTask2bSM, self).__init__()
		self.name = 'HSR Tidy Up Here Task 2b'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:56 y:657, x:189 y:544
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['target_name'])
		_state_machine.userdata.target_name = 'orangecookies'
		_state_machine.userdata.location_name = 'shelf'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:32
			OperatableStateMachine.add('SetPose0',
										hsr_SetBasePoseByTfNameState(tf_name='shelf', service_name='/pose_server/getPose'),
										transitions={'completed': 'Move0'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:456 y:30
			OperatableStateMachine.add('PublishYoloTargetName',
										PublisherStringState(topic='/target_name'),
										transitions={'done': 'FetchObject0'},
										autonomy={'done': Autonomy.Off},
										remapping={'value': 'target_name'})

			# x:266 y:34
			OperatableStateMachine.add('Move0',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'PublishYoloTargetName', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:727 y:240
			OperatableStateMachine.add('SetPoseSearchPersonPoint2',
										hsr_SetBasePoseByTfNameState(tf_name='search_person_point_0', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToSearchPersonPoint2'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:751 y:358
			OperatableStateMachine.add('MoveToSearchPersonPoint2',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'HereYouAre', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:690 y:514
			OperatableStateMachine.add('HereYouAre',
										hsr_SpeakState(sentence='Here you are', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'PassObject'},
										autonomy={'done': Autonomy.Off})

			# x:502 y:613
			OperatableStateMachine.add('PassObject',
										hsr_PassObjectState(service_name='/kinesthetic/wait_open'),
										transitions={'succeeded': 'finished', 'failed': 'PassObject'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:645 y:33
			OperatableStateMachine.add('FetchObject0',
										hsr_FetchObjectDynState(grasp_srv_name='/grasp/service', stop_tf_srv_name='/ork_tf_broadcaster/stop_publish', is_yolo=True),
										transitions={'succeeded': 'SetPoseSearchPersonPoint2', 'failed': 'SetPose1'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_name': 'target_name', 'location_name': 'location_name'})

			# x:30 y:94
			OperatableStateMachine.add('SetPose1',
										hsr_SetBasePoseByTfNameState(tf_name='shelf_1', service_name='/pose_server/getPose'),
										transitions={'completed': 'Move1'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:284 y:111
			OperatableStateMachine.add('Move1',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'FetchObject1', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:23 y:174
			OperatableStateMachine.add('SetPose2',
										hsr_SetBasePoseByTfNameState(tf_name='shelf_2', service_name='/pose_server/getPose'),
										transitions={'completed': 'Move2'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:252 y:191
			OperatableStateMachine.add('Move2',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'FetchObject2', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:38 y:278
			OperatableStateMachine.add('SetPose3',
										hsr_SetBasePoseByTfNameState(tf_name='shelf_3', service_name='/pose_server/getPose'),
										transitions={'completed': 'Move3'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:278 y:291
			OperatableStateMachine.add('Move3',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'FetchObject3', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:22 y:391
			OperatableStateMachine.add('SetPose4',
										hsr_SetBasePoseByTfNameState(tf_name='shelf_4', service_name='/pose_server/getPose'),
										transitions={'completed': 'Move4'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:290 y:382
			OperatableStateMachine.add('Move4',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'FetchObject4', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:465 y:113
			OperatableStateMachine.add('FetchObject1',
										hsr_FetchObjectDynState(grasp_srv_name='/grasp/service', stop_tf_srv_name='/ork_tf_broadcaster/stop_publish', is_yolo=True),
										transitions={'succeeded': 'SetPoseSearchPersonPoint2', 'failed': 'SetPose2'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_name': 'target_name', 'location_name': 'location_name'})

			# x:430 y:196
			OperatableStateMachine.add('FetchObject2',
										hsr_FetchObjectDynState(grasp_srv_name='/grasp/service', stop_tf_srv_name='/ork_tf_broadcaster/stop_publish', is_yolo=True),
										transitions={'succeeded': 'SetPoseSearchPersonPoint2', 'failed': 'SetPose3'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_name': 'target_name', 'location_name': 'location_name'})

			# x:467 y:288
			OperatableStateMachine.add('FetchObject3',
										hsr_FetchObjectDynState(grasp_srv_name='/grasp/service', stop_tf_srv_name='/ork_tf_broadcaster/stop_publish', is_yolo=True),
										transitions={'succeeded': 'SetPoseSearchPersonPoint2', 'failed': 'SetPose4'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_name': 'target_name', 'location_name': 'location_name'})

			# x:463 y:372
			OperatableStateMachine.add('FetchObject4',
										hsr_FetchObjectDynState(grasp_srv_name='/grasp/service', stop_tf_srv_name='/ork_tf_broadcaster/stop_publish', is_yolo=True),
										transitions={'succeeded': 'SetPoseSearchPersonPoint2', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_name': 'target_name', 'location_name': 'location_name'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
