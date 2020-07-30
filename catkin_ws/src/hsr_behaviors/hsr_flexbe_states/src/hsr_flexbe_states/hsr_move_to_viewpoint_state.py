#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Pose, PoseStamped
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from flexbe_core import EventState, Logger
import hsrb_interface
import tf
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
import copy


class hsr_MoveToViewpointState(EventState):
    '''
    Move the HSR base to the 'request' target pose using an Actionlib action.

    ># pose 		Pose	Target pose that the base should reach.

    <= succeeded			The base has succesfully moved to the target pose.
    <= failed				If the robot have selected a viewpoint that the robot have already visited
    '''

    def __init__(self):
        super(hsr_MoveToViewpointState, self).__init__(outcomes=['succeeded', 'failed'], input_keys=['pose'])
        self.cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        self.cli.wait_for_server()
        self._robot = hsrb_interface.Robot()
        self._whole_body = self._robot.get("whole_body")
        self._previous_pose = Pose()

    def execute(self, userdata):
        print("-----------------")
        print("next:")
        print(userdata.pose)
        print("current")
        print(self._previous_pose)
        print("-----------------")
        if self.check_equal_pose(userdata.pose, self._previous_pose) is True:
            return "failed"
        self._previous_pose = copy.deepcopy(userdata.pose)
        # Set goal
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        # Set viewpoint pose
        base_pose = copy.deepcopy(userdata.pose)
        # Set viewpoint height
        arm_lift_height = base_pose.position.z - 1.0
        base_pose.position.z = 0.0
        # Set tilt
        quat = base_pose.orientation
        euler = tf.transformations.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))
        head_tilt_joint = euler[1]
        # Set orientaion
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, euler[2])
        base_pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
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
        self._whole_body.move_to_go()
        self._whole_body.move_to_joint_positions({"arm_lift_joint": arm_lift_height,
                                                  "head_tilt_joint": -head_tilt_joint})
        return 'succeeded'

    def on_enter(self, userdata):
        rospy.set_param("/viewpoint_planner_viewer/start_total_travel_timer", True)

    def on_exit(self, userdata):
        rospy.set_param("/viewpoint_planner_viewer/start_total_travel_timer", False)
        rospy.set_param("/viewpoint_planner_viewer/add_number_of_moves", 1)

    def on_start(self):
        pass

    def on_stop(self):
        pass

    def quaternion_to_euler(self, quaternion):
        e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        return Vector3(x=e[0], y=e[1], z=e[2])

    def check_equal_pose(self, p1, p2):
        if p1.position.x == p2.position.x and p1.position.y == p2.position.y and p1.position.z == p2.position.z:
            if p1.orientation.x == p2.orientation.x and p1.orientation.y == p2.orientation.y:
                if p1.orientation.x == p2.orientation.z and p1.orientation.y == p2.orientation.w:
                    return True
        return False
