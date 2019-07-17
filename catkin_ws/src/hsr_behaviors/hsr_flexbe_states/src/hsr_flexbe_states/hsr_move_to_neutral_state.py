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
import hsrb_interface

class hsr_MoveToNeutralState(EventState):
    '''
    State of passing an object to a person 

    <= succeeded                       Successfully passed the object.
    <= failed                          Passing the object failed.
    '''

    def __init__(self):
        super(hsr_MoveToNeutralState,self).__init__(outcomes=['succeeded', 'failed'])

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
        # Move to neutral 
        #
        try:
            self._whole_body.move_to_neutral()
            self._failed = False
        except Exception as e:
            rospy.logerr(e)
            self._failed = True

    def on_exit(self, userdata):
        pass
    
    def on_start(self):
        pass
    
    def on_stop(self):
        pass
