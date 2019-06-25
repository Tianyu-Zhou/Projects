'''Import dependencies'''
import numpy as np
import train
import test

'''Import DataSet'''
data_train = np.load("data_train.npy")
data_test = np.load("data_test.npy")


train_image_cut = train.cut_image(data_train,500,1500,4000,5000,'train_cut.jpg')
test_image_cut = test.cut_image(data_test,1000,1500,2000,2500,'test_cut.jpg')


(k,red_car_position_train) = train.train(train_image_cut,'train_cut.jpg')
red_car_position_test = test.test(test_image_cut,'test_cut.jpg',k)