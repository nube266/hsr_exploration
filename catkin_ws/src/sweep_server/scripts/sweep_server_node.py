#!/usr/bin/python
#h -*- coding: utf-8 -*-

import hsrb_interface
import rospy
import tf
import sys
from hsrb_interface import geometry
from sweep_server.srv import *

def sweep(req):
    print req.word + req.word
    return sweep_serverResponse(req.word + req.word)

def ints_sweep_server():
    rospy.init_node('sweep_server')
    s = rospy.Service('sweep', sweep_server, sweep)
    print "Ready to sweep."
    rospy.spin()

if __name__ == "__main__":
    ints_sweep_server()
