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
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['target_name'])
		_state_machine.userdata.target_name = 'orangecookies'
		_state_machine.userdata.location_name = 'shelf'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:32
			OperatableStateMachine.add('SetPoseSearchPersonPoint',
										hsr_SetBasePoseByTfNameState(tf_name='shelf', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToSearchPersonPoint'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:357 y:121
			OperatableStateMachine.add('PublishYoloTargetName',
										PublisherStringState(topic='/target_name'),
										transitions={'done': 'FetchObject'},
										autonomy={'done': Autonomy.Off},
										remapping={'value': 'target_name'})

			# x:266 y:34
			OperatableStateMachine.add('MoveToSearchPersonPoint',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'PublishYoloTargetName', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:613 y:227
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

			# x:544 y:384
			OperatableStateMachine.add('HereYouAre',
										hsr_SpeakState(sentence='Here you are', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'PassObject'},
										autonomy={'done': Autonomy.Off})

			# x:494 y:497
			OperatableStateMachine.add('PassObject',
										hsr_PassObjectState(service_name='/kinesthetic/wait_open'),
										transitions={'succeeded': 'finished', 'failed': 'PassObject'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:372 y:212
			OperatableStateMachine.add('FetchObject',
										hsr_FetchObjectDynState(grasp_srv_name='/grasp/service', stop_tf_srv_name='/ork_tf_broadcaster/stop_publish', is_yolo=True),
										transitions={'succeeded': 'SetPoseSearchPersonPoint2', 'failed': 'SetPoseSearchPersonPoint'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_name': 'target_name', 'location_name': 'location_name'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
