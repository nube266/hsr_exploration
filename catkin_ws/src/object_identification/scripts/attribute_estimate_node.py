#!/usr/bin/env python
import os
import numpy as np
import cv2
import rospy
import rospkg 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from object_identification.srv import *
from object_identification.msg import Attribute, AttributeArray

class GetAttribute:
    def __init__(self):
        self.bridge = CvBridge()
        rospack = rospkg.RosPack()
        np_filename = os.path.join(rospack.get_path('object_identification'), 'weights/np_weights.npz')
        #print (np_filename)
        np_data = np.load(np_filename)
        self.w1 = np_data['w1']
        self.w2 = np_data['w2']
        self.w3 = np_data['w3']
        self.b1 = np_data['b1']
        self.b2 = np_data['b2']
        self.b3 = np_data['b3']
        self.attr_str = np_data['attr_str']
        self.srv = rospy.Service('get_attribute', get_attribute, self.get_attribute_srv)

    def make_histogram(self, image):
        hist = np.zeros((26,), dtype=np.float32)
        h, w, _ = image.shape
        for y in range(h):
            for x in range(w):
                r = image[y,x,2]
                g = image[y,x,1]
                b = image[y,x,0]
                idx = self.get_index(r,g,b)
                hist[idx] = hist[idx] + 1.0

        hist = hist/float(h*w)
        return hist

    def get_index(self, r, g, b):
        v_th = 0.2
        s_th = 0.2
        r = int(r)
        g = int(g)
        b = int(b)
        min_v = min(r, g, b)
        max_v = max(r, g, b)
        maxval = 255.0

        v = float(max_v)/maxval
        s = float(max_v-min_v)/maxval
        if max_v == min_v :
            hue = 0.0
        else :
            if max_v == r :
                hue = 60.0 * float(g-b)/float(max_v-min_v) + 0.0
            elif max_v == g :
                hue = 60.0 * float(b-r)/float(max_v-min_v) + 120.0
            else :
                hue = 60.0 * float(r-g)/float(max_v-min_v) + 240.0

        while hue >= 360.0:
            hue = hue - 360.0
        while hue < 0.0 :
            hue = hue + 360.0

        ret = 0
        if v < v_th :
            ret = 0
        elif s < s_th :
            ret = 1
        elif s < (s_th + (1.0-s_th)/2.0):
            ret = 2 + int(hue/30.0)
        else :
            ret = 2 + 12 + int(hue/30.0)

        return ret

    def npsigmoid(self, x):
        x[x<-50]=-50
        x[x>50]=50
        return 1.0/(1.0+np.exp(-x))

    def get_attribute_srv(self, req):
        try :
            cv_image = self.bridge.imgmsg_to_cv2(req.image, "bgr8")
        except CvBridgeError as e:
            print(e)
        #cv2.imshow('test', cv_image)
        #cv2.waitKey()
        hist = self.make_histogram(cv_image)
        hist = hist[np.newaxis, :]
        h = np.dot(hist, self.w1)+self.b1
        h[h<0.0]=0.0
        h = np.dot(h, self.w2)+self.b2
        h[h<0.0]=0.0
        h = np.dot(h, self.w3)+self.b3
        y = self.npsigmoid(h)
        ret = AttributeArray()
        for i in range(self.attr_str.shape[0]):
            work = Attribute()
            work.attribute_name = self.attr_str[i]
            work.score = y[0,i]
            ret.attributes.append(work)
        return get_attributeResponse(ret)

if __name__ == "__main__":
    rospy.init_node('attribute_estimate_node')
    ga = GetAttribute()
    rospy.spin()
