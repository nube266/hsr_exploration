import os 
import glob
import cv2
import numpy as np
#import keras 
#import keras.backend as K
from model import  make_histogram
import math

def npsigmoid(x):
    #x = max(x, -500)
    print x
    x[x<-50]=-50
    x[x>50]=50
    return 1.0/(1.0+np.exp(-x))

if __name__ == '__main__':

    np_data = np.load('np_weights.npz')
    w1 = np_data['w1']
    w2 = np_data['w2']
    w3 = np_data['w3']
    b1 = np_data['b1']
    b2 = np_data['b2']
    b3 = np_data['b3']
    attr_str = np_data['attr_str']

    img1 = cv2.imread('dataset/mixed_images/comb/banana1.png')
    img2 = cv2.imread('dataset/mixed_images/comb/grape1.png')
    img3 = cv2.imread('/host/tmp/database/bowl/target_0000.png')
    img4 = cv2.imread('/host/tmp/database/tomato/target_0000.png')

    for img in [img1, img2, img3, img4]:
        hist = make_histogram(img)
        hist = hist[np.newaxis, :]
        h = np.dot(hist, w1)+b1
        h[h<0.0]=0.0
        h = np.dot(h, w2)+b2
        h[h<0.0]=0.0
        h = np.dot(h, w3)+b3
        y = npsigmoid(h)
        for i in range(11):
            print ("{} {} {}".format(i, attr_str[i], y[0,i]))

