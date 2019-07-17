#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_search_object_state import hsr_SearchObjectState
from hsr_flexbe_states.hsr_speak_dyn_state import hsr_SpeakDynState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 17 2019
@author: ShigemichiMatsuzaki
'''
class HSRDynamicSpeakTestSM(Behavior):
	'''
	Test behavior for 'hsr_SpeakDynState'
	'''


	def __init__(self):
		super(HSRDynamicSpeakTestSM, self).__init__()
		self.name = 'HSR Dynamic Speak Test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:478, x:130 y:478
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:136 y:32
			OperatableStateMachine.add('SearchObject',
										hsr_SearchObjectState(search_point='searching_point_0', search_place_type='floor', service_search_floor='/search_object/search_floor', service_update_threshold='/ork_tf_broadcaster/update_threshold', service_publish_tf='/ork_tf_broadcaster/start_publish', service_stop_tf='/ork_tf_broadcaster/stop_publish', centroid_x_max=1.5, centroid_y_max=1.0, centroid_y_min=-1.0, centroid_z_max=0.1, centroid_z_min=0.0, sleep_time=5.0, is_floor=True),
										transitions={'found': 'Speak', 'notfound': 'finished', 'failed': 'failed'},
										autonomy={'found': Autonomy.Off, 'notfound': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_name': 'object_name'})

			# x:266 y:130
			OperatableStateMachine.add('Speak',
										hsr_SpeakDynState(sentence="I'll bring +", sentence_when_empty="I couldn't recognize it", topic='/talk_request', interrupting=False, queueing=False, language=1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'variable': 'object_name'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
