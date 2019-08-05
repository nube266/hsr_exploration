#!/usr/bin/env python

import rospy
from flexbe_core import EventState
from rospy.exceptions import ROSInterruptException

'''
Created on 15.06.2013

@author: David Conner
'''

class WaitDynState(EventState):
    '''
    Implements a state that can be used to wait on timed process.

    ># wait_time    float   Amount of time to wait minutes.

    <= done                                 Indicates that the wait time has elapsed.

    '''
    def __init__(self):
        '''Constructor'''
        super(WaitDynState, self).__init__(outcomes=['done'], input_keys=['wait_time'])
                
    def execute(self, userdata):
        '''Execute this state'''

        elapsed = rospy.get_rostime() - self._start_time;
        if (elapsed.to_sec() > self._wait * 60.0):
            return 'done'

        
    def on_enter(self, userdata):
        '''Upon entering the state, save the current time and start waiting.'''

        self._wait = userdata.wait_time
        self._start_time = rospy.get_rostime()

        rospy.loginfo('Wait : ' + str(self._wait * 60.0) + ' [sec]')
                
        try:
            self._rate.sleep()
        except ROSInterruptException:
            rospy.logwarn('Skipped sleep.')
