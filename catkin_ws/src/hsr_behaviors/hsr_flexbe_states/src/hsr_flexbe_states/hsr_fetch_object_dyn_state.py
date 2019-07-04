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
from grasp_server.srv import grasp_srv, grasp_srvRequest
from std_srvs.srv import Empty, EmptyRequest

class hsr_FetchObjectDynState(EventState):
    '''
    A state to call grasp_server service given target object name and location name as userdata. 
    Requred package : grasp_server (https://aisl-serv6.aisl.cs.tut.ac.jp:20443/wrs/grasp_server)

    -- grasp_srv_name          String    A name of a service to be called
    -- stop_tf_srv_name      String    The name of a service used for stoping tf from ork_tf_broadcaster (if needed)
    -- is_yolo               Bool      Whether YOLO is used for the object detection
     
    ># target_name           String    Name of the object to be grasped
    ># location_name         String    Name of the place where the object is located

    <= succeeded                       An object was found.
    <= failed                          No object was found.

    
    '''

    def __init__(self, grasp_srv_name='/grasp/service', stop_tf_srv_name='/ork_tf_broadcaster/stop_publish', is_yolo=True):
        super(hsr_FetchObjectDynState, self).__init__(input_keys=['target_name', 'location_name'], outcomes=['succeeded', 'failed'])

        self._grasp_srv_name   = grasp_srv_name
        self._stop_tf_srv_name = stop_tf_srv_name
        self._grasp_server     = ProxyServiceCaller({self._grasp_srv_name : grasp_srv})
        self._stop_tf_server   = ProxyServiceCaller({self._stop_tf_srv_name : Empty})
        self._is_yolo          = is_yolo

    def execute(self, userdata):

        rospy.loginfo('Let\'s Fetch')
        rospy.sleep(1)

        # If succeeded to grasp, return 'succeeded'. Otherwise, false.
        if not  self._failed:
            return 'succeeded'
        else:
            return 'failed'

    def on_enter(self, userdata):
        #
        # Execute the searching motion
        #
        if self._is_yolo:
            self._target_name = 'yolo_' + userdata.target_name
        else:
            self._target_name = userdata.target_name

        self._fetch_place_type = userdata.location_name

        req = grasp_srvRequest(self._target_name, self._fetch_place_type)
        self._failed = False
        try:
            self._srv_result = self._grasp_server.call(self._grasp_srv_name, req)
            if not self._srv_result.is_succeeded:
                self._failed = True
        except Exception as e:
            rospy.logwarn('Failed to call object recognizer:\n\r%s' % str(e))
            self._failed = True
    
    def on_exit(self, userdata):
        pass
    
    def on_start(self):
        pass
    
    def on_stop(self):
        pass
