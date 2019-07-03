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
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBaseState
from hsr_flexbe_states.hsr_search_object_state import hsr_SearchObjectState
from hsr_flexbe_states.hsr_put_object_state import hsr_PutObjectState
from hsr_flexbe_states.hsr_fetch_object_state import hsr_FetchObjectState
from hsr_flexbe_states.hsr_escape_by_twist_state import hsr_EscapeByTwistState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jun 05 2019
@author: ShigemichiMatsuzaki
'''
class HSRTidyUpHere2018SM(Behavior):
	'''
	Demo code of the entire flow of Tidy Up Here in WRS 2018
	'''


	def __init__(self):
		super(HSRTidyUpHere2018SM, self).__init__()
		self.name = 'HSR Tidy Up Here 2018'

		# parameters of this behavior
		self.add_parameter('searching_point', 'searching_point_0')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:87 y:660, x:681 y:294
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:61 y:66
			OperatableStateMachine.add('SetPoseSearchingPoint',
										hsr_SetBasePoseByTfNameState(tf_name='searching_point_0', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToSearchingPoint'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:494 y:489
			OperatableStateMachine.add('MoveToPuttingPoint',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'PutObject', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose2'})

			# x:292 y:126
			OperatableStateMachine.add('MoveToSearchingPoint',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'SearchObject', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:50 y:244
			OperatableStateMachine.add('SearchObject',
										hsr_SearchObjectState(search_point=self.searching_point, search_place_type='floor', service_name='/search_object/search_floor', centroid_x_max=1.5, centroid_y_max=1.0, centroid_y_min=-1.0, centroid_z_max=0.1, centroid_z_min=0.0, sleep_time=5.0),
										transitions={'found': 'FetchObject', 'notfound': 'finished', 'failed': 'failed'},
										autonomy={'found': Autonomy.Off, 'notfound': Autonomy.Off, 'failed': Autonomy.Off})

			# x:719 y:560
			OperatableStateMachine.add('PutObject',
										hsr_PutObjectState(put_place_type='shelf', target_name='soundtoybox', service_name='/grasp/put'),
										transitions={'succeeded': 'finished', 'failed': 'Escape'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:229 y:469
			OperatableStateMachine.add('SetPosePuttingPoint',
										hsr_SetBasePoseByTfNameState(tf_name='toyshelf', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToPuttingPoint'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose2'})

			# x:43 y:398
			OperatableStateMachine.add('FetchObject',
										hsr_FetchObjectState(fetch_place_type='floor', grasp_srv_name='/grasp/service', stop_tf_srv_name='/ork_tf_broadcaster/stop_publish', target_name='closest'),
										transitions={'succeeded': 'SetPosePuttingPoint', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:792 y:447
			OperatableStateMachine.add('Escape',
										hsr_EscapeByTwistState(topic='/hsrb/command_velocity', linear_x=-0.5, linear_y=0.0, angular=0.0, duration=2.0),
										transitions={'completed': 'failed'},
										autonomy={'completed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
