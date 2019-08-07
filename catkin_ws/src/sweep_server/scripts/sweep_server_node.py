#!/usr/bin/python
# h -*- coding: utf-8 -*-

#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
#
# Yusuke Miake
#

import hsrb_interface
import rospy
import math
import tf
import sys
from hsrb_interface import geometry
from sweep_server.srv import *


class Sweep:
    JOINT_POSITIONS_GRASP_OBJECT = {'arm_flex_joint': -1.86,
                                    'arm_roll_joint': 0,
                                    'wrist_flex_joint': -1.33,
                                    'wrist_roll_joint': 1.53}

    JOINT_POSITIONS_NO_GRASP_OBJECT = {'arm_flex_joint': -2.45,
                                       'arm_roll_joint': 0.0,
                                       'wrist_flex_joint': 0.90,
                                       'wrist_roll_joint': 0.0}

    LINEAR_WEIGHT = 80
    ANGULAR_WEIGHT = 80
    IS_GRASP_DISTANCE_THRESHOLD = 0.08
    START_ARM_LIFT_HEIGHT = 0.3

    def __init__(self):
        self._robot = hsrb_interface.Robot()
        self._omni_base = self._robot.get('omni_base')
        self._whole_body = self._robot.get('whole_body')
        self._gripper = self._robot.get('gripper')
        self.listener = tf.TransformListener()
        self._tts = self._robot.get('default_tts')
        self._tts.language = self._tts.ENGLISH

    def sweep(self, req):
        self._target_tf = req.obj
        self._waiting_time = req.waiting_time
        self._sweep_mode = req.sweep_mode
        self._sweep_angular = req.sweep_angular
        self._sweep_distance = req.sweep_distance
        self._sweep_height = req.sweep_height
        self._is_right_move = req.is_right_move
        try:
            exec('res = self.sweep_'+req.loc+'()')
            return sweep_serverResponse(res)
        except:
            return sweep_serverResponse(False)

    def sweep_floor(self):
        try:
            self.listener.waitForTransform(
                self._target_tf, "/base_link", rospy.Time(), rospy.Duration(10))
        except (tf.Exception):
            rospy.logerr('Fail in wait for transform')
            return False

        try:
            exec('self.'+self._sweep_mode+'()')
            return True
        except:
            return False

    def push(self):
        try:
            self.setup_sweep()
            print 'Perform preliminary motion'
            self._omni_base.go_rel(-self._sweep_distance, 0.0, 0.0, 100)
            rospy.sleep(self._waiting_time)
            print 'Lower the robot\'s hand'
            self._whole_body.move_to_joint_positions(
                {'arm_lift_joint': self._sweep_height})
            rospy.sleep(self._waiting_time)
            print 'Do a sweep'
            self._omni_base.go_rel(2*self._sweep_distance, 0.0, 0.0, 100)
            rospy.sleep(self._waiting_time)
            print 'Return to the initial position'
            rospy.sleep(self._waiting_time)
            self._whole_body.move_to_neutral()
            print 'Sweep finish'
            return True
        except Exception as exc:
            self.sweep_error(exc)
            return False

    def rotate(self):
        try:
            self.setup_sweep()
            print 'Perform preliminary motion'
            self._omni_base.go_rel(0.0, 0.0, -self._sweep_angular, 100)
            rospy.sleep(self._waiting_time)
            print 'Lower the robot\'s hand'
            self._whole_body.move_to_joint_positions(
                {'arm_lift_joint': self._sweep_height})
            rospy.sleep(self._waiting_time)
            print 'Do a sweep'
            self._omni_base.go_rel(0.0, 0.0, 2*self._sweep_angular, 100)
            rospy.sleep(self._waiting_time)
            print 'Return to the initial position'
            rospy.sleep(self._waiting_time)
            self._whole_body.move_to_neutral()
            print 'Sweep finish'
            return True
        except Exception as exc:
            self.sweep_error(exc)
            return False

    def lateral(self):
        MOVE_FRONT_DISTANCE_LATERAL = 0.05
        try:
            self.setup_sweep()
            print 'Perform preliminary motion'
            if self._is_right_move:
                self._omni_base.go_rel(0.0, self._sweep_distance, 0.0, 100)
            else:
                self._omni_base.go_rel(0.0, -self._sweep_distance, 0.0, 100)
            self._omni_base.go_rel(MOVE_FRONT_DISTANCE_LATERAL, 0.0, 0.0, 100)
            rospy.sleep(self._waiting_time)
            print 'Lower the robot\'s hand'
            self._whole_body.move_to_joint_positions(
                {'arm_lift_joint': self._sweep_height})
            rospy.sleep(self._waiting_time)
            print 'Do a sweep'
            if self._is_right_move:
                self._omni_base.go_rel(0.0, -2*self._sweep_distance, 0.0, 100)
            else:
                self._omni_base.go_rel(0.0, 2*self._sweep_distance, 0.0, 100)
            rospy.sleep(self._waiting_time)
            print 'Return to the initial position'
            rospy.sleep(self._waiting_time)
            self._whole_body.move_to_neutral()
            print 'Sweep finish'
            if self._is_right_move:
                self._omni_base.go_rel(0.0, self._sweep_distance, 0.0, 100)
            else:
                self._omni_base.go_rel(0.0, -self._sweep_distance, 0.0, 100)
            return True
        except Exception as exc:
            self.sweep_error(exc)
            return False

    def setup_sweep(self):
        print 'Sweep start'
        self._whole_body.linear_weight = Sweep.LINEAR_WEIGHT
        self._whole_body.angular_weight = Sweep.ANGULAR_WEIGHT
        print 'Move the hand to the position of the object'
        self._whole_body.move_end_effector_pose(
            geometry.pose(z=-0.3), self._target_tf)
        rospy.sleep(self._waiting_time)
        print 'Lift the robot lift'
        self._whole_body.move_to_joint_positions(
            {'arm_lift_joint': Sweep.START_ARM_LIFT_HEIGHT})
        rospy.sleep(self._waiting_time)
        print 'Change hand direction'
        if self.get_is_object_grasp():
            self._whole_body.move_to_joint_positions(
                Sweep.JOINT_POSITIONS_GRASP_OBJECT)
        else:
            self._whole_body.move_to_joint_positions(
                Sweep.JOINT_POSITIONS_NO_GRASP_OBJECT)
        rospy.sleep(self._waiting_time)

    def get_is_object_grasp(self):
        # Returns whether robot are gripping an object
        if self._gripper.get_distance() >= Sweep.IS_GRASP_DISTANCE_THRESHOLD:
            return False
        else:
            return True

    def sweep_error(self, exc):
        # Display error message
        self._tts.say('Failed to sweep the object')
        rospy.logerr('Fail to sweep; The error as follows')
        rospy.logerr(exc)
        self._whole_body.move_to_neutral()


if __name__ == "__main__":
    sweep = Sweep()
    srv = rospy.Service('sweep', sweep_server, sweep.sweep)
    print "Ready to sweep"
    rospy.spin()
