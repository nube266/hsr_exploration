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
import hsrb_interface

class hsr_PutObjectState(EventState):
    '''
    A state to call grasp_server service. 
    Requred package : grasp_server (https://aisl-serv6.aisl.cs.tut.ac.jp:20443/wrs/grasp_server)

    -- put_place_type        String    A type of the place where the putting is done {'shelf', 'toyshelf', 'rubbishbin'}
    -- service_name          String    A name of a service to be called
    -- target_name           String    The name of tf on which the robot tries to put

    <= succeeded                       An object was found.
    <= failed                          No object was found.
    '''

    def __init__(self, put_place_type, target_name, service_name='/grasp/put'):
        super(hsr_PutObjectState, self).__init__(outcomes=['succeeded', 'failed'])

        self._put_place_type = put_place_type
        self._service_name     = service_name
        self._target_name      = target_name
        self._put_server     = ProxyServiceCaller({self._service_name : grasp_srv})

        self._whole_body = hsrb_interface.Robot().get('whole_body')

    def execute(self, userdata):
        # If succeeded to grasp, return 'succeeded'. Otherwise, false.
        if not self._failed:
            return 'succeeded'
        else:
            self._whole_body.move_to_neutral()
            return 'failed'

    def on_enter(self, userdata):
        #
        # Execute the searching motion
        rospy.loginfo('Let\'s Put')
        rospy.sleep(1)      #

        req = grasp_srvRequest(self._target_name, self._put_place_type)
        self._failed = False
        try:
            self._srv_result = self._put_server.call(self._service_name, req)
            rospy.loginfo(self._srv_result.is_succeeded)
            if not self._srv_result.is_succeeded:
                self._failed = True
        except Exception as e:
            rospy.logwarn('Failed to call object recognizer:\n\r%s' % str(e))
            self._failed = True
    
    def on_exit(self, userdata):
        pass
    
    def on_start(self):
        pass
    
    def on_stop(self):
        pass
