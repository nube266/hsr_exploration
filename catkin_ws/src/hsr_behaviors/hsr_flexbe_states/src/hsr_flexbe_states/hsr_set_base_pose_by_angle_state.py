#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Pose
from flexbe_core import EventState, Logger

class hsr_SetBasePoseByAngleState(EventState):
	'''
	Set a target 'pose' to reach with the HSR base.
        Orientation is given by a rotation angle along z axis of the map frame (assuming the rotation axis as [0, 0, 1]) 

	-- pose_position_x		Pose	Target pose position along x.
	-- pose_position_y		Pose	Target pose position along y.
	-- pose_orientation_theta	Pose	Target pose orientation along a vector [0,0,1].
        -- is_rad                       Boolean Boolean that indicates whether the unit of the angle value is radians

	#> pose					Pose	Target pose that the base should reach.

	<= completed					The target pose has been set.
	'''

	def __init__(self, pose_position_x, pose_position_y, pose_orientation_theta, is_rad=True):
		super(hsr_SetBasePoseByAngleState,self).__init__(outcomes=['completed'],output_keys=['pose'])
		self._pose = Pose()
		self._pose.position.x = pose_position_x
		self._pose.position.y = pose_position_y

                '''
                If the angle is given in degrees, convert the value to radians
                '''
                rad_angle = pose_orientation_theta
                if not is_rad:
                    rad_angle = pose_orientation_theta * math.pi / 180.0 

                '''
                A quaternion that represents the rotation of theta[rad] along a vector [lambda_x, lambda_y, lambda_z] is given as follows:
                    [lambda_x*sin(theta/2), lambda_y*sin(theta/2), lambda_z*sin(theta/2), cos(theta/2)]

                As here the rotation axis is assumed to be [0, 0, 1], only the third and fourth elements of the quaternion (z and w) matters,
                  and those values are simply given as:
                    z = sin(theta/2)
                    w = cos(theta/2)
                '''

		self._pose.orientation.z = math.sin(rad_angle/2)
		self._pose.orientation.w = math.cos(rad_angle/2)

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
