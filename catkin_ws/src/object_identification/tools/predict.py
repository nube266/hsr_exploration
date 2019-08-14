import os 
import glob
import cv2
import numpy as np
import keras 
import keras.backend as K
from model import make_model, make_histogram
import math

def sigmoid(x):
    x = max(x, -500)
    return 1.0/(1.0+math.exp(-x))

def npsigmoid(x):
    #x = max(x, -500)
    print x
    x[x<-50]=-50
    x[x>50]=50
    return 1.0/(1.0+np.exp(-x))

if __name__ == '__main__':

    attr_dict = {}
    f =  open('model/attr_dict.txt', 'r')
    readlines = f.readlines()
    f.close()
    for line in readlines:
        dat = line.split(',')
        if dat[0] ==  "":
            continue
        attr_dict[dat[0]] = int(dat[1])
    print attr_dict

    use_hist = True
    if use_hist:
        input_shape = (26,)
    else: 
        input_shape = (224,224,3)

    input_data = keras.layers.Input( input_shape )
    model = make_model(input_data, input_shape, len(attr_dict), use_hist)
    model.load_weights('model/model_weight.hdf5')
    img1 = cv2.imread('dataset/mixed_images/comb/banana1.png')
    img2 = cv2.imread('dataset/mixed_images/comb/grape1.png')
    img3 = cv2.imread('/host/tmp/database/bowl/target_0000.png')
    img4 = cv2.imread('/host/tmp/database/tomato/target_0000.png')
    for img in [img1, img2, img3, img4]:
        if use_hist:
            hist = make_histogram(img)
            hist = hist[np.newaxis, :]
            y =  model.predict(hist)
        else :
            img = cv2.resize(img, (224,224))
            img = img.astype(np.float32)/255.0
            img = img[np.newaxis, :, :, :]
            y = model.predict(img)

        for attr_str, num in attr_dict.items():    
            print ("{} : {}".format(attr_str,sigmoid(y[0,num])) )
        print ()


    w1, b1 = model.layers[1].get_weights()
    w1 = np.array(w1)
    b1 = np.array(b1)
    w2, b2 = model.layers[2].get_weights()
    w2 = np.array(w2)
    b2 = np.array(b2)
    w3, b3 = model.layers[3].get_weights()
    w3 = np.array(w3)
    b3 = np.array(b3)
    print w1.shape, b1.shape
    print w2.shape, b2.shape
    print w3.shape, b3.shape
    h = np.dot(hist[0],w1)+b1
    h[h<0.0]=0.0
    h = np.dot(h,w2)+b2
    h[h<0.0]=0.0
    h = np.dot(h,w3)+b3
    h = npsigmoid(h)
    print h
    for attr_str, num in attr_dict.items():    
        print ("{} : {}".format(attr_str,h[num]))

    attr_str_list = []
    for _ in range(len(attr_dict)):
        attr_str_list.append("")
    for attr_str, num in attr_dict.items():
        attr_str_list[num] = attr_str
    attr_str_np = np.array(attr_str_list)
    print attr_str_np
    

    np.savez('np_weights.npz', w1=w1, b1=b1, w2=w2, b2=b2, w3=w3, b3=b3, attr_str=attr_str_np)    
