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
from std_srvs.srv import Empty, EmptyRequest

class hsr_SearchObjectState(EventState):
    '''
    A state to call search_object service used in Tidy Up task. 
    Requred package : search_object (https://aisl-serv6.aisl.cs.tut.ac.jp:20443/wrs/search_object)

    -- search_point               String    A tf name of a searching point
    -- search_place_type          String    A type of the place to be checked {'floor'}
    -- service_search_floor       String    A name of the service to search an object
    -- service_update_threshold   String    A name of the service to be update the threshold of 'ork_tf_broadcaster'
    -- centroid_x_max             float     Threshold of tf to publish
    -- centroid_y_max             float     Threshold of tf to publish
    -- centroid_y_min             float     Threshold of tf to publish
    -- centroid_z_max             float     Threshold of tf to publish
    -- centroid_z_min             float     Threshold of tf to publish

    <= found                          An object was found.
    <= notfound                       An object was found.
    <= failed                         Failed to process necessary programs.
    '''

    def __init__(self, search_point, search_place_type, service_search_floor='/search_object/search_floor', service_update_threshold='/ork_tf_broadcaster/update_threshold', 
                 service_publish_tf='/ork_tf_broadcaster/start_publish', service_stop_tf='/ork_tf_broadcaster/stop_publish',
                 centroid_x_max = 1.5, centroid_y_max = 1.0, centroid_y_min = -1.0,
                 centroid_z_max  = 0.1, centroid_z_min = 0.0, sleep_time = 5.0, is_floor = True):
        super(hsr_SearchObjectState,self).__init__(outcomes=['found', 'notfound', 'failed'], output_keys=['object_name'])

        # Service names etc.
        self._search_point = search_point # The locations to be checked
        self._search_place_type = search_place_type
        self._service_search_floor = service_search_floor
        self._service_publish_tf = service_publish_tf
        self._service_stop_tf = service_stop_tf
        self._service_update_threshold = service_update_threshold

        # Server
        self._search_object_server = ProxyServiceCaller({self._service_search_floor : search_object_srv})
        self._publish_tf = ProxyServiceCaller({self._service_publish_tf : Empty})
        self._stop_tf = ProxyServiceCaller({self._service_stop_tf : Empty})
        self._update_threshold_server = ProxyServiceCaller({self._service_update_threshold : Empty})

        # Parameters
        self._centroid_x_max = centroid_x_max
        self._centroid_y_max = centroid_y_max
        self._centroid_y_min = centroid_y_min
        self._centroid_z_max = centroid_z_max
        self._centroid_z_min = centroid_z_min
        self._centroid_z_min = centroid_z_min
        self._sleep_time     = sleep_time
        self._is_floor       = is_floor

    def execute(self, userdata):
        '''
        Execute the state
        '''
        return self._outcome


    def on_enter(self, userdata):
        # Set and update the parameters
        rospy.set_param("/ork_tf_broadcaster/centroid_x_max", self._centroid_x_max)
        rospy.set_param("/ork_tf_broadcaster/centroid_y_max", self._centroid_y_max)
        rospy.set_param("/ork_tf_broadcaster/centroid_y_min", self._centroid_y_min)
        rospy.set_param("/ork_tf_broadcaster/centroid_z_max", self._centroid_z_max)
        rospy.set_param("/ork_tf_broadcaster/centroid_z_min", self._centroid_z_min)
        rospy.set_param("/ork_tf_broadcaster/is_floor", self._is_floor)
        rospy.set_param("/search_object/sleep_time", self._sleep_time)
        # Update threshold
        req = EmptyRequest()

        try:
            result = self._update_threshold_server.call(self._service_update_threshold, req)
        except Exception as e:
            rospy.logwarn('Failed to call object recognizer:\n\r%s' % str(e))
            self._outcome = 'failed'

#        try:
#            result = self._publish_tf.call(self._service_publish_tf, req)
#        except Exception as e:
#            rospy.logwarn('Failed to call object recognizer:\n\r%s' % str(e))
#            self._outcome = 'failed'

        #
        # Execute the searching motion
        #
        rospy.loginfo('Let\'s search')

        req = search_object_srvRequest('', self._search_place_type, self._search_point)

        self._outcome = 'found'
        try:
            self._srv_result = self._search_object_server.call(self._service_search_floor, req)
            userdata.object_name = self._srv_result.object_name

            if not self._srv_result.is_succeeded:
                self._outcome = 'notfound'
        except Exception as e:
            rospy.logwarn('Failed to call object recognizer:\n\r%s' % str(e))
            self._outcome = 'failed'
    
    def on_exit(self, userdata):
        pass
#        req = EmptyRequest()
#
#        try:
#            result = self._stop_tf.call(self._service_stop_tf, req)
#        except Exception as e:
#            rospy.logwarn('Failed to call object recognizer:\n\r%s' % str(e))
#            self._outcome = 'failed'
    
    def on_stop(self):
        pass
