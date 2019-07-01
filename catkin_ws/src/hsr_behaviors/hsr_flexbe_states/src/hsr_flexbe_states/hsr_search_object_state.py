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
from search_object.srv import search_object_srv, search_object_srvRequest

class hsr_SearchObjectState(EventState):
    '''
    A state to call search_object service used in Tidy Up task. 
    Requred package : search_object (https://aisl-serv6.aisl.cs.tut.ac.jp:20443/wrs/search_object)

    -- search_point            String    A tf name of a searching point
    -- search_place_type       String    A type of the place to be checked {'floor'}
    -- service_name            String    A name of the service to be called
    -- centroid_x_max          float     Threshold of tf to publish
    -- centroid_y_max          float     Threshold of tf to publish
    -- centroid_y_min          float     Threshold of tf to publish
    -- centroid_z_max          float     Threshold of tf to publish
    -- centroid_z_min          float     Threshold of tf to publish

    <= succeeded                       An object was found.
    <= failed                          No object was found.
    '''

    def __init__(self, search_point, search_place_type, service_name='/search_object/search_floor',
                 centroid_x_max = 1.5, centroid_y_max = 1.0, centroid_y_min = -1.0,
                 centroid_z_max  = 0.1, centroid_z_min = 0.0):
        super(hsr_SearchObjectState,self).__init__(outcomes=['succeeded', 'failed'])

        self._search_point = search_point # The locations to be checked
        self._search_place_type = search_place_type
        self._service_name = service_name
        rospy.set_param("/ork_tf_broadcaster/centroid_x_max", centroid_x_max)
        rospy.set_param("/ork_tf_broadcaster/centroid_y_max", centroid_y_max)
        rospy.set_param("/ork_tf_broadcaster/centroid_y_min", centroid_y_min)
        rospy.set_param("/ork_tf_broadcaster/centroid_z_max", centroid_z_max)
        rospy.set_param("/ork_tf_broadcaster/centroid_z_min", centroid_z_min)
        self._search_object_server = ProxyServiceCaller({self._service_name : search_object_srv})

    def execute(self, userdata):
        '''
        Execute the state
        '''
        rospy.loginfo('Let\'s search')

        # Wait for a second
        rospy.sleep(1)
        # If an object is found, return 'succeeded'
        if not  self._failed:
            rospy.loginfo('I found an object')
            return 'succeeded'
        else:
            rospy.loginfo('I could\'nt find any objects')
            return 'failed'


    def on_enter(self, userdata):
        pass
        #
        # Execute the searching motion
        #
        req = search_object_srvRequest('', self._search_place_type, self._search_point)

        self._failed = False
        try:
            self._srv_result = self._search_object_server.call(self._service_name, req)
            rospy.loginfo(self._srv_result)
        except Exception as e:
            rospy.logwarn('Failed to call object recognizer:\n\r%s' % str(e))
            self._failed = True
    
    def on_exit(self, userdata):
        pass
    
    def on_start(self):
        pass
    
    def on_stop(self):
        pass
