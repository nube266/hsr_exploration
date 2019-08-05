#!/usr/bin/env python
#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
# 
# Shigemichi Matsuzaki
#
import rospy
import time
from flexbe_core import EventState
from flexbe_core import Logger
from flexbe_core.proxy import ProxyPublisher

class hsr_CheckElapsedTimeState(EventState):
    '''
    Check the elapsed time and if time is up.
    -- time_limit      float      Time limit (minutes)
    -- margin          float      The margin to keep from the time limit (minutes)

    ># start_time      float      System time that indicates the time of starting the behavior [seconds] (expected to be set by the initial state)
    #> offset          float      Remaining time to the time limit

    <= time_remains               Outcome that indicates time still remains
    <= time_up                    Outcome that indicates time is up
    '''

    def __init__(self, time_limit, margin):
        '''
        Constructor
        '''
        super(hsr_CheckElapsedTimeState, self).__init__(input_keys=['start_time'], output_keys=['offset'], outcomes=['time_remains', 'time_up'])

        self._time_limit = time_limit
        self._margin = margin

    def execute(self, userdata):
        # Measure the elapsed time
        start_time = userdata.start_time
        now = time.time()
        elapsed_time = (now - start_time) / 60.0
        userdata.offset = self._time_limit - elapsed_time

        rospy.loginfo('elapsed_time' + str(elapsed_time))

        # If elapsed time is more the given time_limit, return 'time_up'
        if elapsed_time < self._time_limit - self._margin:
            return 'time_remains'
        else:
            return 'time_up'
