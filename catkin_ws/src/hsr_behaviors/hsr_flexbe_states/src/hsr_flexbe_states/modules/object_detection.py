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
    def __init__(self):
        self._bbox_datas = []
        self._target_names = []
        target_name = rospy.get_param("/search_table_server/TARGET_NAME", None)
        if target_name is not None:
            self._target_names.append(target_name)
        loop_count = 1
        while True:
            param_name = "/search_table_server/TARGET_NAME" + str(loop_count)
            loop_count = loop_count + 1
            target_name = rospy.get_param(param_name, None)
            if target_name is None:
                break
            else:
                self._target_names.append(target_name)
        bbox_topic_name = rospy.get_param("/search_table_server/BOUNDING_BOX_TOPIC_NAME", "/darknet_ros/bounding_boxes")
        rospy.Subscriber(bbox_topic_name, BoundingBoxes, self.sub_callback)

    def main(self):
        start_time = time.time()
        search_time = rospy.get_param("/search_table_server/SEARCH_TIME", 10)
        r = rospy.Rate(10)  # 10Hz
        self._bbox_datas = []
        while time.time() - start_time <= search_time:
            r.sleep()
            if self._bbox_datas is None:
                continue
            for target_name in self._target_names:
                if target_name in self._bbox_datas:
                    print("Target object found")
                    return True
        print("Not found")
        return False

    def sub_callback(self, data):
        for bounding_box in data.bounding_boxes:
            self._bbox_datas.append(bounding_box.Class)
            pass
