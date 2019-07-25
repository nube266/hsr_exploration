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

class hsr_FetchObjectInterfaceState(EventState):
    '''
    State to set parameters for 'HSR FetchObject' behavior.

    -- centroid_x_max             float     Threshold of tf to publish
    -- centroid_y_max             float     Threshold of tf to publish
    -- centroid_y_min             float     Threshold of tf to publish
    -- centroid_z_max             float     Threshold of tf to publish
    -- centroid_z_min             float     Threshold of tf to publish
    -- sleep_time                 float     Time to wait in SearchObject 
    -- is_floor                   bool      Type of TF published from 'ork_tf_broadcaster'

    #> centroid_x_max             float     Threshold of tf to publish
    #> centroid_y_max             float     Threshold of tf to publish
    #> centroid_y_min             float     Threshold of tf to publish
    #> centroid_z_max             float     Threshold of tf to publish
    #> centroid_z_min             float     Threshold of tf to publish
    #> sleep_time                 float     Time to wait in SearchObject 
    #> is_floor                   bool      Type of TF published from 'ork_tf_broadcaster'

    <= done                       Done
    '''

    def __init__(self, centroid_x_max, centroid_y_max, centroid_y_min, centroid_z_max, centroid_z_min, sleep_time, is_floor):

        super(hsr_FetchObjectInterfaceState,self).__init__(outcomes=['done'],
                output_keys=['centroid_x_max', 'centroid_y_max', 'centroid_y_min', 'centroid_z_max', 'centroid_z_min', 'sleep_time', 'is_floor'])

        self._centroid_x_max = centroid_x_max
        self._centroid_y_max = centroid_y_max
        self._centroid_y_min = centroid_y_min
        self._centroid_z_max = centroid_z_max
        self._centroid_z_min = centroid_z_min
        self._sleep_time     = sleep_time
        self._is_floor       = is_floor

    def execute(self, userdata):
        '''
        Execute the state
        '''
        return 'done'


    def on_enter(self, userdata):
        userdata.centroid_x_max = self._centroid_x_max
        userdata.centroid_y_max = self._centroid_y_max
        userdata.centroid_y_min = self._centroid_y_min
        userdata.centroid_z_max = self._centroid_z_max
        userdata.centroid_z_min = self._centroid_z_min
        userdata.sleep_time     = self._sleep_time    
        userdata.is_floor       = self._is_floor      
    
    def on_exit(self, userdata):
        pass
    
    def on_stop(self):
        pass
