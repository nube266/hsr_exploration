#!/usr/bin/python
# h -*- coding: utf-8 -*-

#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
#
# Yusuke Miake
#
import tf
from geometry_msgs.msg import Vector3


def judge_reach_goal(robot_pose, goal_pose, xy_tolerance, yaw_tolerance):
    robot_position = (robot_pose.pos.x, robot_pose.pos.y)
    goal_position = (goal_pose.position.x, goal_pose.position.y)
    robot_orientation = robot_pose.ori
    goal_orientation = goal_pose.orientation
    robot_yaw = get_yaw_by_quaternion(robot_orientation)
    goal_yaw = get_yaw_by_quaternion(goal_orientation)
    x_min = goal_position[0] - xy_tolerance
    x_max = goal_position[0] + xy_tolerance
    y_min = goal_position[1] - xy_tolerance
    y_max = goal_position[1] + xy_tolerance
    yaw_min = goal_yaw - yaw_tolerance
    yaw_max = goal_yaw + yaw_tolerance
    if x_min <= robot_position[0] and robot_position[0] <= x_max:
        if y_min <= robot_position[1] and robot_position[1] <= y_max:
            if yaw_min <= robot_yaw and robot_yaw <= yaw_max:
                return True
    return False


def get_yaw_by_quaternion(quaternion):
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return e[2]
