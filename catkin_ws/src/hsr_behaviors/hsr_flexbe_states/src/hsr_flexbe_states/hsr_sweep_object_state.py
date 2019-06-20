#!/usr/bin/env python
#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
# 
# Yusuke Miake
#
import rospy
from flexbe_core import EventState
from flexbe_core import Logger
from flexbe_core.proxy import ProxyServiceCaller
from sweep_server.srv import sweep_server, sweep_serverRequest
from std_srvs.srv import Empty, EmptyRequest

class hsr_SweepObjectState(EventState):
    '''
    A state to call sweep_server service. 

    -- service_name          String    A name of a service to be called
    -- target_name           String    The name of tf which the robot tries to sweep
    

    <= succeeded                       Sweep was succeeded.
    <= failed                          Sweep was failed.

    '''

    def __init__(self, sweep_srv_name='/sweep/service', stop_tf_srv_name='/ork_tf_broadcaster/stop_publish', target_name):
        super(hsr_SweepObjectState, self).__init__(outcomes=['succeeded', 'failed'])

        self._sweep_srv_name   = sweep_srv_name
        self._stop_tf_srv_name = stop_tf_srv_name
        self._target_name      = target_name
        self._sweep_server     = ProxyServiceCaller({self._sweep_srv_name : sweep_srv})

    def execute(self, userdata):

        rospy.loginfo('Let\'s Sweep')
        rospy.sleep(1)

        # If succeeded to sweep, return 'succeeded'. Otherwise, 'failed'.
        if not  self._failed:
            return 'succeeded'
        else:
            return 'failed'

    def on_enter(self, userdata):
        self._failed = False
    
    def on_exit(self, userdata):
        req = EmptyRequest()
    
    def on_start(self):
        pass
    
    def on_stop(self):
        pass
