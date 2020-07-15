#!/usr/bin/python
# h -*- coding: utf-8 -*-

#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
#
# Yusuke Miake
#
import time
import rospy
from darknet_ros_msgs.msg import BoundingBoxes


class object_detector:
    def __init__(self, target_object_1, target_object_2, target_object_3,
                 timeout, bounding_box_topic):
        self._bbox_datas = []
        self._target_names = []
        self._search_time = timeout
        if target_object_1 != "":
            self._target_names.append(target_object_1)
        if target_object_2 != "":
            self._target_names.append(target_object_2)
        if target_object_3 != "":
            self._target_names.append(target_object_3)
        bbox_topic_name = bounding_box_topic
        rospy.Subscriber(bbox_topic_name, BoundingBoxes, self.sub_callback)

    def main(self):
        start_time = time.time()
        r = rospy.Rate(10)  # 10Hz
        self._bbox_datas = []
        rospy.set_param("/darknet_ros/enable_darknet_ros", True)
        while time.time() - start_time <= self._search_time:
            r.sleep()
            if self._bbox_datas is None:
                continue
            for target_name in self._target_names:
                if target_name in self._bbox_datas:
                    print("Target object found")
                    rospy.set_param("/darknet_ros/enable_darknet_ros", False)
                    return True
        print("Not found")
        rospy.set_param("/darknet_ros/enable_darknet_ros", False)
        return False

    def sub_callback(self, data):
        for bounding_box in data.bounding_boxes:
            self._bbox_datas.append(bounding_box.Class)
            pass
