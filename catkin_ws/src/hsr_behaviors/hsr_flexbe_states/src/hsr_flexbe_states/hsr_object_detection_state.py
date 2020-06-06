#!/usr/bin/env python
#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
#
# Yusuke Miake
#
import rospy
from flexbe_core import EventState
from flexbe_core import Logger
from flexbe_core.proxy import ProxyServiceCaller
from modules.object_detection import object_detector


class hsr_ObjectDetectionState(EventState):
    '''
    State that execute object detection using yolo.

    <= found                      Target found
    <= not found                  Target not found
    '''

    def __init__(self, target_object_1="", target_object_2="", target_object_3="",
                 timeout=10.0, bounding_box_topic="/darknet_ros/bounding_boxes"):
        super(hsr_ObjectDetectionState, self).__init__(outcomes=["found", "not_found"])
        self._object_detector = object_detector(target_object_1, target_object_2, target_object_3, timeout, bounding_box_topic)

    def execute(self, userdata):
        rospy.loginfo("Search target")
        is_found = self._object_detector.main()
        if is_found:
            rospy.loginfo("Target found")
            return "found"
        else:
            rospy.loginfo("Target not found")
            return "not_found"

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
