#!/usr/bin/env python
from flexbe_core import EventState
from geometry_msgs.msg import Pose, Quaternion, Vector3
from collections import OrderedDict
import tf
import roslib.packages
import random
import json
import os


class hsr_GetObjectPoseState(EventState):
    '''
    Get a pose in target_object_area(Please see hsr_flexbe_states/config/target_object_area.json)

    ># pose 		    Pose	    Randomly determined pose

    <= succeeded
    '''

    def __init__(self, pose_list_path="/root/HSR/catkin_ws/src/hsr_behaviors/hsr_flexbe_states/config/target_object_area.json",
                 save_data_path="/root/HSR/catkin_ws/src/hsr_behaviors/hsr_flexbe_states/src/hsr_flexbe_states/data/log.csv"):
        super(hsr_GetObjectPoseState, self).__init__(outcomes=["succeeded"], output_keys=["pose"])
        self._save_data_path = save_data_path
        self._object_pose_list = []
        with open(pose_list_path) as f:
            dic = json.load(f, object_pairs_hook=OrderedDict)
            for area_name in dic.keys():
                area = dic[area_name]
                x_min = area["x_min"]
                x_max = area["x_max"]
                y_min = area["y_min"]
                y_max = area["y_max"]
                z_min = area["z_min"]
                z_max = area["z_max"]
                pose = Pose()
                pose.position.x = (x_max + x_min) / 2
                pose.position.y = (y_max + y_min) / 2
                pose.position.z = (z_max + z_min) / 2
                pose.orientation = self.euler_to_quaternion(Vector3(0.0, 0.0, 0.0))
                self._object_pose_list.append([area_name, pose])
        if os.path.isfile(self._save_data_path) is False:
            with open(self._save_data_path, mode="w") as f:
                f.write("area_name,date,success,total_search_time,total_viewpoint_planning_time,number_of_moves,total_travel_time\n")

    def execute(self, userdata):
        name_pose = self._object_pose_list.pop(0)
        with open(self._save_data_path, mode="a") as f:
            f.write("{0},".format(name_pose[0]))
        userdata.pose = name_pose[1]
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
