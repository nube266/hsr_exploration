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
from hsr_flexbe_states.hsr_analyse_command_state import hsr_AnalyseCommandState
from flexbe_states.publisher_string_state import PublisherStringState
from hsr_flexbe_states.hsr_set_base_pose_by_tf_name_dyn_state import hsr_SetBasePoseByTfNameDynState
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBaseState
from hsr_flexbe_states.hsr_fetch_object_state import hsr_FetchObjectDynState
from hsr_flexbe_states.hsr_speak_state import hsr_SpeakState
from hsr_flexbe_states.hsr_pass_object_state import hsr_PassObjectState
from flexbe_states.subscriber_state import SubscriberState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 12 2019
@author: Shigemichi Matsuzaki
'''
class HSRBringMeC2502SM(Behavior):
	'''
	Demo code of the entire flow of Bring Me in C2-502
	'''


	def __init__(self):
		super(HSRBringMeC2502SM, self).__init__()
		self.name = 'HSR Bring Me C2-502'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:32
			OperatableStateMachine.add('SetPoseSearchPersonPoint',
										hsr_SetBasePoseByTfNameState(tf_name='search_person_point_0', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToSearchPersonPoint'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:271 y:129
			OperatableStateMachine.add('Analyse',
										hsr_AnalyseCommandState(service_name='/wrs_semantics/bring_me_instruction'),
										transitions={'succeeded': 'SetPoseFetchLocation', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'command': 'message', 'object_name': 'object_name', 'location_name': 'location_name'})

			# x:250 y:261
			OperatableStateMachine.add('PublishYoloTargetName',
										PublisherStringState(topic='/target_name'),
										transitions={'done': 'FetchObject'},
										autonomy={'done': Autonomy.Off},
										remapping={'value': 'object_name'})

			# x:488 y:134
			OperatableStateMachine.add('SetPoseFetchLocation',
										hsr_SetBasePoseByTfNameDynState(service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToFetchLocation'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tf_name': 'location_name', 'pose': 'pose'})

			# x:728 y:134
			OperatableStateMachine.add('MoveToFetchLocation',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'PublishYoloTargetName', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:530 y:280
			OperatableStateMachine.add('FetchObject',
										hsr_FetchObjectDynState(grasp_srv_name='/grasp/service', stop_tf_srv_name='/ork_tf_broadcaster/stop_publish', is_yolo=True),
										transitions={'succeeded': 'SetPoseSearchPersonPoint2', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_name': 'object_name', 'location_name': 'location_name'})

			# x:266 y:34
			OperatableStateMachine.add('MoveToSearchPersonPoint',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'AskForCommand', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:535 y:32
			OperatableStateMachine.add('AskForCommand',
										hsr_SpeakState(sentence='How may I help you?', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'Wait'},
										autonomy={'done': Autonomy.Off})

			# x:702 y:282
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

			# x:648 y:450
			OperatableStateMachine.add('HereYouAre',
										hsr_SpeakState(sentence='Here you are', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'PassObject'},
										autonomy={'done': Autonomy.Off})

			# x:639 y:548
			OperatableStateMachine.add('PassObject',
										hsr_PassObjectState(service_name='/kinesthetic/wait_open'),
										transitions={'succeeded': 'AskForCommand', 'failed': 'PassObject'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:28 y:125
			OperatableStateMachine.add('ListenCommand',
										SubscriberState(topic='/sr_res', blocking=True, clear=True),
										transitions={'received': 'Analyse', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:712 y:25
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=1.5),
										transitions={'done': 'ListenCommand'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
