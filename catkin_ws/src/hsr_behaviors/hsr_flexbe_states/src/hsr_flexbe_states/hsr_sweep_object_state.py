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

    -- target_name           String    The name of tf which the robot tries to sweep
    -- sweep_place_type      String    A type of the place where the sweeping is done (Only the 'floor' is implemented)
    -- sweep_srv_name        String    A name of a service to be called
    -- waiting_time          float32   Time between actions  (It has to be more than 3 seconds at the time of simulation)
    -- sweep_mode            String    How to sweep {'push':Move back and forth, 'rotate': Rotate, 'lateral': Move left and right}
    -- sweep_angular         float32   Angle to rotation in sweep
    -- sweep_distance        float32   Distance to move back and forth in sweep [m]
    -- sweep_height          float32   Height in sweep [m]
    -- is_right_move         bool      Move to the right if true, to the left if false

    <= succeeded                       Sweep was succeeded.
    <= failed                          Sweep was failed.

    '''

    def __init__(self, target_name, sweep_place_type='floor', sweep_srv_name='/sweep', waiting_time=3.0, sweep_mode = 'push',
                 sweep_angular=1.0, sweep_distance=0.25, sweep_height=0.05, is_right_move=True, stop_tf_srv_name='/ork_tf_broadcaster/stop_publish'):
        super(hsr_SweepObjectState, self).__init__(outcomes=['succeeded', 'failed'])

        self._target_name      = target_name
        self._sweep_place_type = sweep_place_type
        self._sweep_srv_name   = sweep_srv_name
        self._waiting_time     = waiting_time
        self._sweep_mode       = sweep_mode
        self._sweep_angular    = sweep_angular
        self._sweep_distance   = sweep_distance
        self._sweep_height     = sweep_height
        self._is_right_move    = is_right_move
        self._stop_tf_srv_name = stop_tf_srv_name
        self._sweep_server     = ProxyServiceCaller({self._sweep_srv_name : sweep_server})
        self._stop_tf_server   = ProxyServiceCaller({self._stop_tf_srv_name : Empty})

    def execute(self, userdata):

        rospy.loginfo('Let\'s Sweep')
        rospy.sleep(1)

        # If succeeded to sweep, return 'succeeded'. Otherwise, 'failed'.
        if not self._failed:
            return 'succeeded'
        else:
            return 'failed'

    def on_enter(self, userdata):
        #
        # Execute the searching motion
        #
        req = sweep_serverRequest(self._target_name, self._sweep_place_type, self._waiting_time, self._sweep_mode,
                                  self._sweep_angular, self._sweep_distance, self._sweep_height, self._is_right_move)
        self._failed = False
        try:
            self._srv_result = self._sweep_server.call(self._sweep_srv_name, req)
            rospy.loginfo(self._srv_result)
            self._failed = not self._srv_result.is_succeeded
        except Exception as e:
            rospy.logwarn('Failed to call object recognizer:\n\r%s' % str(e))
            self._failed = True
    
    def on_exit(self, userdata):
        pass
        '''
        req = EmptyRequest()
        try:
            self._srv_result = self._stop_tf_server.call(self._stop_tf_srv_name, req)
        except Exception as e:
            rospy.logwarn('Failed to call object recognizer:\n\r%s' % str(e))
            self._failed = True
        '''
    
    def on_start(self):
        pass
    
    def on_stop(self):
        pass
