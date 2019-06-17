#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from pose_server.srv import GetPose, GetPoseRequest

class hsr_SetBasePoseByTfNameDynState(EventState):
    '''
    Set a target 'pose' to reach with the HSR base, using the tf name registered in pose_server.
    This state depends on pose_server package https://aisl-serv6.aisl.cs.tut.ac.jp:20443/wrs/pose_server
    
    -- service_name	  String	The name of the service that provides the mapping of name to pose

    ># tf_name	          String	The name of the tf of the target location
    
    #> pose		  Pose	        Target pose that the base should reach.
    
    <= completed					The target pose has been set.
    '''

    def __init__(self, service_name='/pose_server/getPose'):
        super(hsr_SetBasePoseByTfNameDynState,self).__init__(input_keys=['tf_name'], output_keys=['pose'], outcomes=['completed'])
        self._service_name = service_name
        self._get_pose_server = ProxyServiceCaller({self._service_name : GetPose})
    
    def execute(self, userdata):
        #
        # Substitute the pose value
        #
        userdata.pose = self._pose

        return 'completed'
    
    def on_enter(self, userdata):
        #
        # Initialize the request variable
        #
        self._tf_name      = userdata.tf_name
        req = GetPoseRequest(self._tf_name)
        self._failed = False
        try:
            #
            # Execute the service call
            #
            self._srv_result = self._get_pose_server.call(self._service_name, req)
            rospy.loginfo(self._srv_result)
        except Exception as e:
            rospy.logwarn('Failed to call object recognizer:\n\r%s' % str(e))
            self._failed = True

        #
        # Substitute the pose value
        #   * The returned value is PoseStamped, so result.pose has 'header' and 'pose'.
        #
        self._pose = self._srv_result.pose.pose
    
    def on_exit(self, userdata):
        pass
    
    def on_start(self):
        pass
    
    def on_stop(self):
        pass
