#!/usr/bin/env python
from flexbe_core import EventState
from geometry_msgs.msg import Pose, Quaternion, Vector3
import tf
import roslib.packages
import random
import json


class hsr_GetRandomTargetPoseState(EventState):
    '''
    Get random pose in target_object_area(Please see hsr_flexbe_states/config/target_object_area.json)

    ># output		Pose	Randomly determined pose

    <= succeeded
    '''

    def __init__(self):
        super(hsr_GetRandomTargetPoseState, self).__init__(outcomes=["succeeded"], output_keys=["pose"])

    def execute(self, userdata):
        pose = Pose()
        path = roslib.packages.get_pkg_dir("hsr_flexbe_states") + "/config/target_object_area.json"
        with open(path) as f:
            dic = json.load(f)
            name, area = random.choice(list(dic.items()))
            x_min = area["x_min"]
            x_max = area["x_max"]
            y_min = area["y_min"]
            y_max = area["y_max"]
            z_min = area["z_min"]
            z_max = area["z_max"]
            pose.position.x = random.uniform(x_min, x_max)
            pose.position.y = random.uniform(y_min, y_max)
            pose.position.z = random.uniform(z_min, z_max)
            pose.orientation = self.euler_to_quaternion(Vector3(0.0, 0.0, 0.0))
            print(">>>>>>>>>>>>")
            print("name: {0}".format(name))
            print("pose: {0}".format(pose))
            print("<<<<<<<<<<<<")
        userdata.pose = pose
        return "succeeded"

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

    def euler_to_quaternion(self, euler):
        q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
