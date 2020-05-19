#!/usr/bin/env python
#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
#
# Yusuke Miake
#
import rospy
import time
import actionlib
from geometry_msgs.msg import Pose, PoseStamped
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from modules.judge_reach_goal import judge_reach_goal
from modules.calculation_travel_distance import calculation_travel_distance
import hsrb_interface
from flexbe_core import EventState
from flexbe_core import Logger
from flexbe_core.proxy import ProxyServiceCaller


class hsr_MoveStateForGazebo(EventState):
    '''
    Movement state for gazebo

    <= succeeded                        Move was succeeded.
    <= failed                           Move was failed.

    '''

    def __init__(self):
        super(hsr_MoveStateForGazebo, self).__init__(outcomes=["succeeded", "failed"], input_keys=["request"])
        self._robot = hsrb_interface.Robot()
        self._omni_base = self._robot.get("omni_base")
        self._cli = actionlib.SimpleActionClient("/move_base/move", MoveBaseAction)
        self._cli.wait_for_server()
        self._timeout = int(rospy.get_param("/search_table_server/MOVE_TIMEOUT"))
        self._xy_tolerance = rospy.get_param("/search_table_server/XY_GOAL_TOLERANCE")
        self._yaw_tolerance = rospy.get_param("/search_table_server/YAW_GOAL_TOLERANCE")

    def execute(self, userdata):
        goal_pose = userdata.request
        self._start_time = time.time()
        is_sent_goal = self.move(goal_pose)
        if is_sent_goal is False:
            return "succeeded"
        self.wait(goal_pose)
        if self._cli.get_state() == 0 or self._cli.get_state() == 2 or self._cli.get_state() == 3:
            rospy.loginfo("[Action move_base/move] succeeded.")
            return "succeeded"
        else:
            rospy.loginfo("[Action move_base/move] failed.")
            self._cli.cancel_all_goals()
            return "failed"

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

    def move(self, goal_pose):
        rospy.set_param("viewer_for_search_table//add_number_of_moves", 1)
        robot_pose = self._omni_base.get_pose()
        if judge_reach_goal(robot_pose, goal_pose, self._xy_tolerance, self._yaw_tolerance):
            rospy.loginfo("Has already arrived at the goal")
            return False
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = goal_pose
        self._cli.send_goal(goal)
        rospy.loginfo("Sent goal")
        return True

    def wait(self, goal_pose):
        past_pose = self._omni_base.get_pose()
        while time.time() - self._start_time <= self._timeout:
            time.sleep(1.0)
            robot_pose = self._omni_base.get_pose()
            d = calculation_travel_distance(past_pose, robot_pose)
            past_pose = robot_pose
            rospy.set_param("/viewer_for_search_table/add_travel_distance", d)
            if judge_reach_goal(robot_pose, goal_pose, self._xy_tolerance, self._yaw_tolerance):
                break
        print("move elapsed time:\t{}".format(time.time() - self._start_time))
        rospy.set_param("/viewer_for_search_table/add_travel_time", time.time() - self._start_time)
        if(self._cli.get_state() == 1):
            self._cli.cancel_goal()
        while self._cli.get_state() != 0 and self._cli.get_state() != 2 and self._cli.get_state() != 3 and self._cli.get_state() != 4:
            # Wait for the state to change (0:PENDING 2:PREEMPTED 3:SUCCEEDED)
            # 4: ABORTED
            pass
