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
from grasp_server.srv import open_drawer_srv, open_drawer_srvRequest

class hsr_OpenDrawerState(EventState):
    '''
    A state to call grasp_server service. 
    Requred package : grasp_server (https://aisl-serv6.aisl.cs.tut.ac.jp:20443/wrs/grasp_server)

    -- open_srv_name    String    A name of a service to be called
    -- height           String    The name of tf which the robot tries to grasp

    <= succeeded                       Drawer opening succeeded
    <= failed                          Drawer opening failed
    '''

    def __init__(self, open_srv_name='/grasp/open_drawer', height=0):
        super(hsr_OpenDrawerState, self).__init__(outcomes=['succeeded', 'failed'])

        self._open_srv_name   = open_srv_name
        self._height = height
        self._open_drawer_server = ProxyServiceCaller({self._open_srv_name : open_drawer_srv})

    def execute(self, userdata):
        # If succeeded to grasp, return 'succeeded'. Otherwise, false.
        if not  self._failed:
            return 'succeeded'
        else:
            return 'failed'

    def on_enter(self, userdata):
        rospy.loginfo('Let\'s open the drawer ' + str(self._height))
        rospy.sleep(1)

        #
        # Execute the drawer opening
        #
        req = open_drawer_srvRequest(self._height)
        self._failed = False
        try:
            self._srv_result = self._open_drawer_server.call(self._open_srv_name, req)
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
