#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from flexbe_core import EventState, Logger

class hsr_SetBasePoseState(EventState):
	'''
	Set a target 'pose' to reach with the HSR base.

	-- pose_position_x		Pose	Target pose position along x.
	-- pose_position_y		Pose	Target pose position along y.
	-- pose_orientation_z	Pose	Target pose orientation along z.
	-- pose_orientation_w	Pose	Target pose orientation along w.

	#> pose					Pose	Target pose that the base should reach.

	<= completed					The target pose has been set.
	'''

	def __init__(self, pose_position_x, pose_position_y, pose_orientation_z, pose_orientation_w):
		super(hsr_SetBasePoseState,self).__init__(outcomes=['completed'],output_keys=['pose'])
		self._pose = Pose()
		self._pose.position.x = pose_position_x
		self._pose.position.y = pose_position_y
		self._pose.orientation.z = pose_orientation_z
		self._pose.orientation.w = pose_orientation_w

	def execute(self, userdata):
		userdata.pose = self._pose
		return 'completed'

	def on_enter(self, userdata):
		pass

	def on_exit(self, userdata):
		pass

	def on_start(self):
		pass

	def on_stop(self):
		pass
