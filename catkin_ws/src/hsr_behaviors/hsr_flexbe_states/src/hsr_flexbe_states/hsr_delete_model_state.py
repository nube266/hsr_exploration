#!/usr/bin/env python
#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
#
# Yusuke Miake
#
import rospy
from flexbe_core import EventState
from gazebo_msgs.srv import DeleteModel


class hsr_DeleteModelState(EventState):
    '''
    Delete model

    -- model_name   String              The name of the model to be deleted
    -- srv_name     String              service name

    <= succeeded                        Delete succeeded.

    '''

    def __init__(self, model_name="book_1", srv_name="/gazebo/delete_model"):
        super(hsr_DeleteModelState, self).__init__(outcomes=["succeeded"])
        self._srv_name = srv_name
        self._model_name = model_name
        rospy.wait_for_service(self._srv_name)
        self._service = rospy.ServiceProxy(self._srv_name, DeleteModel)

    def execute(self, userdata):
        rospy.loginfo("Delete model")
        self._service(self._model_name)
        return "succeeded"

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
