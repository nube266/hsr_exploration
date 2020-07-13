#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Pose, PoseStamped
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from flexbe_core import EventState, Logger
import hsrb_interface


class hsr_MoveToViewpointState(EventState):
    '''
    Move the HSR base to the 'request' target pose using an Actionlib action.

    ># request		Pose	Target pose that the base should reach.

    <= succeeded			The base has succesfully moved to the target pose.
    <= failed				The base could not move to the target pose.
    '''

    def __init__(self):
        super(hsr_MoveToViewpointState, self).__init__(outcomes=['succeeded', 'failed'], input_keys=['pose'])
        self.cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        self._robot = hsrb_interface.Robot()
        self._whole_body = self._robot.get("whole_body")

    def execute(self, userdata):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        base_pose = userdata.pose
        arm_lift_height = base_pose.position.z - 1.0
        base_pose.position.z = 0.0
        print("-----------------")
        print("base_pose:")
        print(base_pose)
        print("arm_lift_height: {0}".format(arm_lift_height))
        print("-----------------")
        goal.pose = base_pose
        send_goal = MoveBaseGoal()
        send_goal.target_pose = goal
        self.cli.send_goal(send_goal)
        rospy.loginfo("[Action move_base/move] sent goal.")
        self.cli.wait_for_result()
        if self.cli.get_state() != GoalStatus.SUCCEEDED:
            rospy.loginfo("[Action move_base/move] failed.")
            self.cli.cancel_all_goals()
            return 'failed'
        rospy.loginfo("[Action move_base/move] succeeded.")
        rospy.loginfo("Change the height of the viewpoint")
        self._whole_body.move_to_joint_positions({"arm_lift_joint": arm_lift_height})  # HSR B height is 1.0[m]
        return 'succeeded'

    def on_enter(self, userdata):
        self.cli.wait_for_server()

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
