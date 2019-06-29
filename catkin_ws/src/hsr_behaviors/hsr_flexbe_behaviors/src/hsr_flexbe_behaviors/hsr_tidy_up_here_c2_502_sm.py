#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.subscriber_state import SubscriberState
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBaseState
from hsr_flexbe_states.hsr_search_object_state import hsr_SearchObjectState as hsr_flexbe_states__hsr_SearchObjectState
from hsr_flexbe_states.hsr_fetch_object_state import hsr_FetchObjectState
from hsr_flexbe_states.hsr_set_base_pose_by_tf_name_state import hsr_SetBasePoseByTfNameState
from hsr_flexbe_states.hsr_put_object_state import hsr_PutObjectState
from hsr_flexbe_states.hsr_speak_state import hsr_SpeakState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jun 05 2019
@author: ShigemichiMatsuzaki
'''
class HSRTidyUpHereC2502SM(Behavior):
	'''
	Demo code of the entire flow of Tidy Up Here in C2-502
	'''


	def __init__(self):
		super(HSRTidyUpHereC2502SM, self).__init__()
		self.name = 'HSR Tidy Up Here C2-502'

		# parameters of this behavior
		self.add_parameter('searching_point', 'searching_point_0')
		self.add_parameter('putting_point', 'rubbishbin')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:87 y:660, x:805 y:508
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:22 y:127
			OperatableStateMachine.add('Ask',
										hsr_SpeakState(sentence='How may I help you?', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'Listen'},
										autonomy={'done': Autonomy.Off})

			# x:345 y:221
			OperatableStateMachine.add('MoveToSearchingPoint',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'SearchObject', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:214 y:304
			OperatableStateMachine.add('SearchObject',
										hsr_flexbe_states__hsr_SearchObjectState(search_point='searching_point_0', search_place_type='floor', service_name='/search_object/search_floor', centroid_x_max=1.5, centroid_y_max=1.0, centroid_y_min=-1.0, centroid_z_max=0.1, centroid_z_min=0.0),
										transitions={'succeeded': 'FetchObject', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:211 y:425
			OperatableStateMachine.add('FetchObject',
										hsr_FetchObjectState(fetch_place_type='floor', grasp_srv_name='/grasp/service', stop_tf_srv_name='/ork_tf_broadcaster/stop_publish', target_name='closest'),
										transitions={'succeeded': 'SetPosePuttingPoint', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:362 y:469
			OperatableStateMachine.add('SetPosePuttingPoint',
										hsr_SetBasePoseByTfNameState(tf_name='rubbishbin', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToPuttingPoint'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:579 y:618
			OperatableStateMachine.add('MoveToPuttingPoint',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'PutObject', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:85 y:533
			OperatableStateMachine.add('PutObject',
										hsr_PutObjectState(put_place_type='shelf', target_name='put_rubbishbin', service_name='/grasp/put'),
										transitions={'succeeded': 'SetPoseSearchingPoint', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:42 y:197
			OperatableStateMachine.add('SetPoseSearchingPoint',
										hsr_SetBasePoseByTfNameState(tf_name='searching_point_0', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToSearchingPoint'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:258 y:81
			OperatableStateMachine.add('Speak',
										hsr_SpeakState(sentence='Ok', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'SetPoseSearchingPoint'},
										autonomy={'done': Autonomy.Off})

			# x:43 y:49
			OperatableStateMachine.add('Listen',
										SubscriberState(topic='/sr_res', blocking=True, clear=True),
										transitions={'received': 'Speak', 'unavailable': 'Listen'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
