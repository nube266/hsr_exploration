#!/usr/bin/env python
import math
from geometry_msgs.msg import Pose, Quaternion, Vector3
import tf
from flexbe_core import EventState


class hsr_SetBasePoseState(EventState):
    '''
    Pass the base pose defined in the argument to the next state

    ># output		Pose	Pose to pass to the next state
    -- x			float	x coordinate
    -- y			float	y coordinate
    -- yaw			float   yaw[deg] (-180 < yaw <= 180)

    <= succeeded			No branch
    '''

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        super(hsr_SetBasePoseState, self).__init__(outcomes=["succeeded"], output_keys=["pose"])
        self._pose = Pose()
        self._pose.position.x = x
        self._pose.position.y = y
        self._pose.position.z = 0.0
        self._pose.orientation = self.euler_to_quaternion(Vector3(0.0, 0.0, yaw * math.pi / 180))

    def execute(self, userdata):
        userdata.pose = self._pose
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
