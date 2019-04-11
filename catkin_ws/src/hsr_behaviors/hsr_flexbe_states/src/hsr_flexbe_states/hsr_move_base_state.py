#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Pose, PoseStamped
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from flexbe_core import EventState, Logger

class hsr_MoveBaseState(EventState):
	'''
	Move the HSR base to the 'request' target pose using an Actionlib action.

	># request		Pose	Target pose that the base should reach.

	<= succeeded			The base has succesfully moved to the target pose.
	<= failed				The base could not move to the target pose.
	'''

	def __init__(self):
		super(hsr_MoveBaseState,self).__init__(outcomes=['succeeded','failed'],input_keys=['request'])
		self.cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

	def execute(self, userdata):
		goal = PoseStamped()
		goal.header.stamp = rospy.Time.now()
		goal.header.frame_id = "map"
		goal.pose = userdata.request
		send_goal = MoveBaseGoal()
		send_goal.target_pose = goal
		self.cli.send_goal(send_goal)
		rospy.loginfo("[Action move_base/move] sent goal.")
		self.cli.wait_for_result()
		if self.cli.get_state() == GoalStatus.SUCCEEDED:
			rospy.loginfo("[Action move_base/move] succeeded.")
			return 'succeeded'
		else:
			rospy.loginfo("[Action move_base/move] failed.")
			self.cli.cancel_all_goals()
			return 'failed'

	def on_enter(self, userdata):
		self.cli.wait_for_server()

	def on_exit(self, userdata):
		pass

	def on_start(self):
		pass

	def on_stop(self):
		pass
