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

class hsr_CheckElapsedGlobalTimeState(EventState):
    '''
    Check the elapsed time and if time is up, using rosparam so that it is not affected by shutting down the behavior.
    -- time_limit      float      Time limit (minutes)
    -- margin          float      The margin to keep from the time limit (minutes)

    #> offset          float      Remaining time to the time limit

    <= time_remains               Outcome that indicates time still remains
    <= time_up                    Outcome that indicates time is up
    '''

    def __init__(self, time_limit, margin):
        '''
        Constructor
        '''
        super(hsr_CheckElapsedTimeGlobalState, self).__init__(output_keys=['offset'], outcomes=['time_remains', 'time_up'])

        self._time_limit = time_limit
        self._margin = margin

    def execute(self, userdata):
        # Measure the elapsed time
        start_time = 0.0
        rospy.get_param("/hsr_state/start_time", start_time)

        now = time.time()
        elapsed_time = (now - start_time) / 60.0
        userdata.offset = self._time_limit - elapsed_time

        rospy.loginfo('elapsed_time' + str(elapsed_time))

        # If elapsed time is more the given time_limit, return 'time_up'
        if elapsed_time < self._time_limit - self._margin:
            return 'time_remains'
        else:
            return 'time_up'
