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
from std_srvs.srv import Empty, EmptyRequest
import hsrb_interface

class hsr_PassObjectState(EventState):
    '''
    State of passing an object to a person 

    <= succeeded                       Successfully passed the object.
    <= failed                          Passing the object failed.
    '''

    def __init__(self, service_name='/kinesthetic/wait_open'):
        super(hsr_PassObjectState,self).__init__(outcomes=['succeeded', 'failed'])

        self._service_name = service_name
        self._pass_server  = ProxyServiceCaller({self._service_name : Empty}) 

        # 
        # Robot interface
        # 
        self._robot = hsrb_interface.Robot()
        self._whole_body = self._robot.get('whole_body')

    def execute(self, userdata):
        '''
        Execute the state
        '''
        rospy.loginfo('Let\'s search')

        # Wait for a second
        rospy.sleep(1)
        # If an object is found, return 'succeeded'
        if not  self._failed:
            return 'succeeded'
        else:
            return 'failed'


    def on_enter(self, userdata):
        #
        # Hold out the object
        #
        self._whole_body.move_to_joint_positions({'wrist_flex_joint': -1.13,
                                                 'arm_flex_joint': -0.35})

        #
        # Wait for a person to receive
        #
        req = EmptyRequest()

        self._failed = False
        try:
            self._srv_result = self._pass_server.call(self._service_name, req)
            rospy.loginfo(self._srv_result)
        except Exception as e:
            rospy.logwarn('Failed to call object recognizer:\n\r%s' % str(e))
            self._failed = True
        
        self._whole_body.move_to_neutral()
    
    def on_exit(self, userdata):
        pass
    
    def on_start(self):
        pass
    
    def on_stop(self):
        pass
