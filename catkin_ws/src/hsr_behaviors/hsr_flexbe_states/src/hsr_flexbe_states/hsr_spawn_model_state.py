#!/usr/bin/env python
#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
#
# Yusuke Miake
#
import rospy
from flexbe_core import EventState
import subprocess


class hsr_SpawnModelState(EventState):
    '''
    Spawn gazebo model

    -- model_name       String          Select model name in hsr_search_object
    -- model_format     String          Select sdf or urdf

    <= succeeded                        Spawn succeeded.

    '''

    def __init__(self, model_path="/root/HSR/catkin_ws/src/hsr_object_search_world/models/book_1/model.sdf",
                 model_name="book_1", model_format="sdf", x=0.0, y=0.0, z=0.0):
        super(hsr_SpawnModelState, self).__init__(outcomes=["succeeded"])
        self._model_path = model_path
        self._model_name = model_name
        self._model_format = model_format
        self._x = x
        self._y = y
        self._z = z

    def execute(self, userdata):
        rospy.loginfo("Spawn model")
        cmd = "rosrun gazebo_ros spawn_model -file {0} -{1} -x {2} -y {3} -z {4} -model {5}".format(self._model_path, self._model_format, self._x, self._y, self._z, self._model_name)
        subprocess.call(cmd.split())
        return "succeeded"

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
