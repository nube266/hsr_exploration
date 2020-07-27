#!/usr/bin/env python
#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
#
# Yusuke Miake
#
import rospy
from flexbe_core import EventState
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


class hsr_SpawnModelToInputState(EventState):
    '''
    Spawn gazebo model (pose; input_keys)

    -- model_path   String      Model path(sdf file)
    -- model_name   String      model_name

    ># pose 		Pose	    Model pose to spawn

    <= succeeded                Spawn succeeded.

    '''

    def __init__(self, model_path="/root/HSR/catkin_ws/src/hsr_object_search_world/models/cup_blue/model.sdf", model_name="cup_blue"):
        super(hsr_SpawnModelToInputState, self).__init__(outcomes=["succeeded"], input_keys=["pose"])
        self._model_path = model_path
        self._model_name = model_name

    def execute(self, userdata):
        rospy.loginfo("Spawn model")
        with open(self._model_path, "r") as f:
            sdff = f.read()
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        spawn_model_prox = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        spawn_model_prox(self._model_name, sdff, "", userdata.pose, "map")
        return "succeeded"

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
