import os 
import glob
import cv2
import numpy as np
import keras 
import keras.backend as K
from model import make_model, make_histogram
import pickle

from keras.callbacks import LearningRateScheduler

def step_decay(epoch):
    x = 1e-4
    if epoch >= 2000: x = 1e-5
    return x

lr_decay = LearningRateScheduler(step_decay)

def make_attr_dict():
    txt_list = sorted(glob.glob('dataset/attributes/*txt'))

    attr_data = {}
    num = 0

    for txt_file in txt_list:
        #if txt_file.find('objectname') != -1 or txt_file.find('place') != -1 or txt_file.find('size') != -1:
        #    continue
        if txt_file.find('color') == -1 :
            continue 
        f = open(txt_file, 'r')
        data = f.readlines()
        f.close()
        for dat in data:
            dat = dat.replace('\r','').replace('\n','')
            attr_data[dat] = num
            num = num + 1

    #print (attr_data)
    return attr_data


def read_imagedata(list_filename, attr_dict, use_hist):
    f = open(list_filename, 'r')
    lines = f.readlines()
    f.close()

    image_list = []
    label_list = []
    for line in lines:
        dat = line.replace(" ","").replace('\r','').replace('\n','').split(',')
        if len(dat) == 0 or dat[0] == "" :
            continue
        image_file = 'dataset/mixed_images/comb/{}.png'.format(dat[0])

        image = cv2.imread(image_file)

        if use_hist:
            hist = make_histogram(image)
            image_list.append(hist)
        else :
            image = cv2.resize(image, (224,224))
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = image.astype(np.float32)
            image_list.append(image/255.0)


        label = np.zeros((len(attr_dict),) , dtype=np.int32)
        for attr_str in dat[1:]:
            if attr_str == '' :
                continue
            if attr_str in attr_dict:
                index = attr_dict[attr_str]
                label[index] = 1
        label_list.append(label)
    return np.array(image_list), np.array(label_list)

def mymetric(y_true, y_pred):
    #return K.mean(K.cast(K.equal(y_true, K.round(y_pred)), 'float32'))
    return K.mean(K.cast(K.equal(y_true, K.round(K.sigmoid(y_pred))), 'float32'))
    #return K.mean(K.abs(y_true-K.sigmoid(y_pred)))

def myloss(y_true, y_pred):
    return K.binary_crossentropy(output=y_pred, target=y_true, from_logits=True)

if __name__ == '__main__':

    batch_size = 24
    attr_dict = make_attr_dict()
    print (attr_dict)
    print (len(attr_dict))

    use_hist = True

    train_image, train_label = read_imagedata('dataset/list/my_attributes_list_drink_cup_train.txt', attr_dict, use_hist)
    test_image, test_label = read_imagedata('dataset/list/my_attributes_list_drink_cup_test.txt', attr_dict, use_hist)
    if use_hist :
        input_shape = (26, )
    else :
        input_shape = (224,224,3)
    input_data = keras.layers.Input( input_shape )

    model = make_model(input_data, input_shape, len(attr_dict), use_hist)

    model.compile(optimizer=keras.optimizers.Adam(lr = 1e-4), loss=myloss,  metrics=[mymetric])
    tb_cb = keras.callbacks.TensorBoard(log_dir='./result', batch_size=batch_size)
    model.fit(train_image, train_label, batch_size=batch_size, epochs=3000, shuffle = True, validation_data=(test_image, test_label), callbacks=[tb_cb, lr_decay])
    
    with open('model/attr_dict.txt', 'w') as f:
        for attr_str, num in attr_dict.items():
            f.write('{},{}\n'.format(attr_str,num))

    model.save_weights('model/model_weight.hdf5')

    """
    img = cv2.imread('dataset/mixed_images/comb/banana1.png')
    img = cv2.resize(img, (224,224))
    img = img.astype(np.float32)/255.0
    img = img[np.newaxis, :, :, :]
    y = model.predict(img)
    for attr_str, num in attr_dict.items():
        print ("{} : {}".format(attr_str,y[0,num]) )
    """

