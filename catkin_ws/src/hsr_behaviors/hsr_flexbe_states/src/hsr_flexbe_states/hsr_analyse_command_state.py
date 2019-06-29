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
from wrs_semantics.srv import Object_and_location, Object_and_locationRequest

class hsr_AnalyseCommandState(EventState):
    '''
    A state to analyse a user command from e.g., Listen state, and return object name and its location

    -- service_name            String    A name of the service to be called

    ># command                 String    A sentence of a command from the user

    #> object_name             String    An object name
    #> object_location         String    A place where the object is located

    <= succeeded                       Analysis succeeded.
    <= failed                          Analysis failed.
    '''

    def __init__(self, service_name='/wrs_semantics/bring_me_instruction'):
        super(hsr_AnalyseCommandState,self).__init__(input_keys=['command'], output_keys=['object_name', 'location_name'], outcomes=['succeeded', 'failed'])

        self._service_name = service_name

        self._get_object_location_srv = ProxyServiceCaller({self._service_name : Object_and_location})

    def execute(self, userdata):
        '''
        Execute the state
        '''
        # If an object is found, return 'succeeded'
        if not  self._failed:
            return 'succeeded'
        else:
            return 'failed'


    def on_enter(self, userdata):
        pass
        #
        # Analyse the command
        #
        req = Object_and_locationRequest(userdata.command.message)

        self._failed = False
        try:
            self._srv_result = self._get_object_location_srv.call(self._service_name, req)
            userdata.object_name = self._srv_result.object_name
            userdata.location_name = self._srv_result.location_name
#
            rospy.loginfo(self._srv_result.object_name + ' ' + self._srv_result.location_name)
        except Exception as e:
            rospy.logwarn('Failed to call object recognizer:\n\r%s' % str(e))
            self._failed = True
    
    def on_exit(self, userdata):
        pass
    
    def on_start(self):
        pass
    
    def on_stop(self):
        pass
