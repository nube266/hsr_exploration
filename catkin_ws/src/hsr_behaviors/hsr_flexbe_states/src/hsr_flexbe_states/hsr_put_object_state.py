#!/usr/bin/env python
#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
# 
# Shigemichi Matsuzaki
#
import rospy
from flexbe_core import EventState
from flexbe_core import Logger
from flexbe_core.proxy import ProxyServiceCaller
from grasp_server.srv import grasp_srv, grasp_srvRequest

class hsr_PutObjectState(EventState):
    '''
    A state to call grasp_server service. 
    Requred package : grasp_server (https://aisl-serv6.aisl.cs.tut.ac.jp:20443/wrs/grasp_server)

    -- fetch_place_type      String    A type of the place where the grasping is done {'floor', 'diningtable', ...}
    -- service_name          String    A name of a service to be called
    -- target_name           String    The name of tf which the robot tries to grasp

    <= succeeded                       An object was found.
    <= failed                          No object was found.
    '''

    def __init__(self, put_place_type, service_name, target_name):
        super(hsr_PutObjectState, self).__init__(outcomes=['succeeded', 'failed'])

        self._put_place_type = put_place_type
        self._service_name     = service_name
        self._target_name      = target_name
        self._put_server     = ProxyServiceCaller({self._service_name : grasp_srv})

    def execute(self, userdata):

        rospy.loginfo('Let\'s Put')
        rospy.sleep(1)

        # If succeeded to grasp, return 'succeeded'. Otherwise, false.
        if not  self._failed:
            return 'succeeded'
        else:
            return 'failed'

    def on_enter(self, userdata):
        #
        # Execute the searching motion
        #
        req = grasp_srvRequest(self._target_name, self._put_place_type)
        self._failed = False
        try:
            self._srv_result = self._grasp_server.call(self._service_name, req)
            rospy.loginfo(self._srv_result)
        except Exception as e:
            rospy.logwarn('Failed to call object recognizer:\n\r%s' % str(e))
            self._failed = True
    
    def on_exit(self, userdata):
        pass
    
    def on_start(self):
        pass
    
    def on_stop(self):
        pass
