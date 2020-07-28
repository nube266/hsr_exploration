#!/usr/bin/env python
from flexbe_core import EventState
from geometry_msgs.msg import Pose, Quaternion, Vector3
import tf
import roslib.packages
import random
import json


class hsr_GetRandomRobotPose(EventState):
    '''
    Get random pose in target_object_area(Please see hsr_flexbe_states/config/target_object_area.json)

    ># output		Pose	Randomly determined pose

    <= succeeded
    '''

    def __init__(self):
        super(hsr_GetRandomRobotPose, self).__init__(outcomes=["succeeded"], output_keys=["pose"])

    def execute(self, userdata):
        path = roslib.packages.get_pkg_dir("hsr_flexbe_states") + "/config/robot_accessible_area.json"
        with open(path) as f:
            dic = json.load(f)
            accessible_area = dic["accessible_area"]
            x_min = accessible_area["x_min"]
            x_max = accessible_area["x_max"]
            y_min = accessible_area["y_min"]
            y_max = accessible_area["y_max"]
            pose = Pose()
            rand_x = 0
            rand_y = 0
            while True:
                rand_x = random.uniform(x_min, x_max)
                rand_y = random.uniform(y_min, y_max)
                accessible_flag = True
                for key in dic.keys():
                    if "impassable_area" in key:
                        impassable_area = dic[key]
                        rand_x_min = impassable_area["x_min"]
                        rand_x_max = impassable_area["x_max"]
                        rand_y_min = impassable_area["y_min"]
                        rand_y_max = impassable_area["y_max"]
                        if rand_x_min <= rand_x <= rand_x_max and rand_y_min <= rand_y <= rand_y_max:
                            accessible_flag = False
                if accessible_flag is True:
                    break
            pose.position.x = rand_x
            pose.position.y = rand_y
            pose.position.z = 0.0
            rand_yaw = random.uniform(0, 3.14)
            pose.orientation = self.euler_to_quaternion(Vector3(0.0, 0.0, rand_yaw))
            userdata.pose = pose
            print(">>>>>>>>>>>>")
            print("pose: {0}".format(pose))
            print("<<<<<<<<<<<<")
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
