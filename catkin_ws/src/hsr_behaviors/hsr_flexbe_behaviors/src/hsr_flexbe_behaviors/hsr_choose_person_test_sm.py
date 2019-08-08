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
from hsr_flexbe_states.hsr_choose_person_state import hsr_ChoosePersonState
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBaseState
from hsr_flexbe_states.hsr_pass_object_state import hsr_PassObjectState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Aug 07 2019
@author: ShigemichiMatsuzaki
'''
class HSRChoosePersonTestSM(Behavior):
	'''
	Test behavior of 'hsr_ChoosePersonState', which subscribes to a result of person detection (tracking), and decide which side the person is, left or right.
	'''


	def __init__(self):
		super(HSRChoosePersonTestSM, self).__init__()
		self.name = 'HSR Choose Person Test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:250 y:679, x:529 y:524
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('SetPoseDeliveryArea',
										hsr_SetBasePoseByTfNameState(tf_name='deliveryarea', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToDeliveryArea'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:174 y:203
			OperatableStateMachine.add('SpeakLeft',
										hsr_SpeakState(sentence='You are on the left', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'SetPosePassLeft'},
										autonomy={'done': Autonomy.Off})

			# x:401 y:206
			OperatableStateMachine.add('SpeakRight',
										hsr_SpeakState(sentence='You are on the right', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'SetPosePassRight'},
										autonomy={'done': Autonomy.Off})

			# x:278 y:113
			OperatableStateMachine.add('ChoosePerson',
										hsr_ChoosePersonState(topic_name='/monocular_person_following/target_pose'),
										transitions={'left': 'SpeakLeft', 'right': 'SpeakRight'},
										autonomy={'left': Autonomy.Off, 'right': Autonomy.Off})

			# x:113 y:300
			OperatableStateMachine.add('SetPosePassLeft',
										hsr_SetBasePoseByTfNameState(tf_name='deliveryarea_left', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToThePerson'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:397 y:299
			OperatableStateMachine.add('SetPosePassRight',
										hsr_SetBasePoseByTfNameState(tf_name='deliveryarea_right', service_name='/pose_server/getPose'),
										transitions={'completed': 'MoveToThePerson'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:289 y:391
			OperatableStateMachine.add('MoveToThePerson',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'SpeakHereYouAre', 'failed': 'SpeakHereYouAre'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:292 y:586
			OperatableStateMachine.add('Pass',
										hsr_PassObjectState(service_name='/kinesthetic/wait_open'),
										transitions={'succeeded': 'finished', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:251 y:29
			OperatableStateMachine.add('MoveToDeliveryArea',
										hsr_MoveBaseState(),
										transitions={'succeeded': 'SpeakBeforeChoose', 'failed': 'SetPoseDeliveryArea'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose'})

			# x:477 y:32
			OperatableStateMachine.add('SpeakBeforeChoose',
										hsr_SpeakState(sentence='Please wave', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'ChoosePerson'},
										autonomy={'done': Autonomy.Off})

			# x:285 y:480
			OperatableStateMachine.add('SpeakHereYouAre',
										hsr_SpeakState(sentence='Here you are', topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'Pass'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
