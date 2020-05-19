#!/usr/bin/env python
import rospy
import actionlib
from tmc_msgs.msg import TalkRequestAction, TalkRequestGoal
from actionlib_msgs.msg import GoalStatus
from flexbe_core import EventState, Logger

class hsr_TalkRequestState(EventState):
	'''
	Request the HSR to say 'talk_sentence' using an Actionlib action.

	-- talk_sentence	string	Requested sentence.

	<= succeeded				The HSR has successfully said the requested sentence.
	<= failed					The HSR could not say the requested sentence.
	'''

	def __init__(self, talk_sentense):
		super(hsr_TalkRequestState,self).__init__(outcomes=['succeeded','failed'])
		self.cli = actionlib.SimpleActionClient('/talk_request_action', TalkRequestAction)
		self.talk_sentense = talk_sentense


	def execute(self, userdata):

		goal = TalkRequestGoal()
		goal.data.sentence = self.talk_sentense
		self.cli.send_goal(goal)
		rospy.loginfo("[Action talk_request_action] sent goal.")
		self.cli.wait_for_result()
		if self.cli.get_state() == GoalStatus.SUCCEEDED:
			rospy.loginfo(self.talk_sentense)
			rospy.loginfo("[Action talk_request_action] succeeded.")
			return 'succeeded'
		else:
			rospy.loginfo("[Action talk_request_action] failed.")
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
