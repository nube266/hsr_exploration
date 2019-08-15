import keras 
import numpy as np

def make_histogram(image):
    hist = np.zeros((26,), dtype=np.float32)
    h, w, _ = image.shape
    for y in range(h):
        for x in range(w):
            r = image[y,x,2]
            g = image[y,x,1]
            b = image[y,x,0]
            idx = get_index(r,g,b)
            hist[idx] = hist[idx] + 1.0
    hist = hist/float(h*w)
    return hist

def get_index(r, g, b):
    v_th = 0.2
    s_th = 0.2
    r = int(r)
    g = int(g)
    b = int(b)
    min_v = min(r, g, b)
    max_v = max(r, g, b)
    #print r, g, b, min_v, max_v
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

def make_model(input_data, input_shape, output_size, use_hist):

    # It isn't achieved the target performance using Resnet152, but, Xception can achieved.
    #pretrain_network = keras.applications.resnet.ResNet152(include_top=False, weights='imagenet', input_shape=input_shape, pooling='avg')
    if use_hist==False: 
        pretrain_network = keras.applications.xception.Xception(include_top=False, weights='imagenet', input_shape=input_shape, pooling='avg')
        pretrain_network.trainable = False
        #print (pretrain_network.summary())

    h_num = 1000
    #decay_lambda = 0.001
    decay_lambda = 0.0001

    if use_hist:
        h = input_data
    else :
        h = pretrain_network(input_data)

    h = keras.layers.Dense(h_num, activation=keras.activations.relu, kernel_regularizer=keras.regularizers.l2(decay_lambda))(h)
    h = keras.layers.Dense(h_num, activation=keras.activations.relu, kernel_regularizer=keras.regularizers.l2(decay_lambda))(h)
    output_data = keras.layers.Dense(output_size, kernel_regularizer=keras.regularizers.l2(decay_lambda))(h)
    #h = keras.layers.Dense(h_num, activation=keras.activations.relu, kernel_regularizer=keras.regularizers.l2(decay_lambda), kernel_initializer=keras.initializers.RandomNormal(),
    #                       bias_initializer=keras.initializers.Zeros())(h)
    #h = keras.layers.Dense(h_num, activation=keras.activations.relu, kernel_regularizer=keras.regularizers.l2(decay_lambda), kernel_initializer=keras.initializers.RandomNormal(),
    #                       bias_initializer=keras.initializers.Zeros())(h)
    #output_data = keras.layers.Dense(output_size, kernel_regularizer=keras.regularizers.l2(decay_lambda), kernel_initializer=keras.initializers.RandomNormal(),
    #                                 bias_initializer=keras.initializers.Zeros())(h)
    model = keras.Model(input_data, output_data)
    print (model.summary())
    return model

