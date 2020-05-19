#!/usr/bin/python
# h -*- coding: utf-8 -*-

#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
#
# Yusuke Miake
#
from math import sqrt, pow


def calculation_travel_distance(past_pose, current_pose):
    p_x = past_pose.pos.x
    p_y = past_pose.pos.y
    c_x = current_pose.pos.x
    c_y = current_pose.pos.y
    distance = sqrt(pow(p_x - c_x, 2) + pow(p_y - c_y, 2))
    return distance
