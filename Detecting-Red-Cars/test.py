'''Import dependencies'''
import numpy as np
import matplotlib.pyplot as plt
import colorsys
from PIL import Image,ImageDraw
import random
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score
from sklearn.neighbors import KNeighborsClassifier

'''----------------define function-----------------'''

'''show red car'''
def show_red_car(image,position):
    img1 = Image.open(image)
    len_red_car = len(position)
    red_car = ImageDraw.Draw(img1) 
    for i in range(len_red_car):
        red_car.rectangle((position[i][0]-10,position[i][1]-10,position[i][0]+10,position[i][1]+10),outline = (0,0,255))
    img1.show()
    return img1

'''cut the image'''
def cut_image(image,min_x,max_x,min_y,max_y,new_image_name):
    image_cut = []
    image_cut_array = []
    for i in range(min_x,max_x):
        for j in range(min_y,max_y):
            image_cut.append(image[i][j])
        image_cut_array.append(image_cut)
        image_cut = []
    image_cut_array = np.array(image_cut_array)
#    plt.figure()
#    plt.imshow(image_cut_array)
    plt.imsave(new_image_name,image_cut_array)
    return image_cut_array
    
'''array spilt'''     
def array_spilt(origin_array,column_cut_len,row_cut_len):
    sample_array = []
    sample_position_array = []
    row_array = np.vsplit(origin_array, len(origin_array)//row_cut_len)
    row_array = np.array(row_array)
    for i in range(len(origin_array)//row_cut_len):
        column_array = np.hsplit(row_array[i],len(origin_array[0])//column_cut_len)
        sample_position_array.append(column_array)
        for j in range(len(origin_array[0])//column_cut_len):
            sample_array.append(column_array[j])
    sample_array = np.array(sample_array)
    sample_position_array = np.array(sample_position_array)
    return sample_array,sample_position_array

'''get red car'''
def cut_figure(data,mid_x,mid_y):
    red_car = []
    red_car_cut = []
    red_car_cut_array = []
    for i in range(len(mid_x)):
        for j in range(mid_x[i]-5, mid_x[i]+5):
            for k in range(mid_y[i]-5,mid_y[i]+5):
                red_car_cut.append(data[j][k])
            red_car_cut_array.append(red_car_cut)
            red_car_cut = []
        red_car_cut_array = np.array(red_car_cut_array)
        red_car.append(red_car_cut_array)
        red_car_cut_array = []
    red_car = np.array(red_car)
    return red_car  

'''get random number'''
def get_random_number(value_min,value_max,num,value_not_in):
    random_point_array = []
    for i in range(num):
        a = random.randint(value_min, value_max)
        b = random.randint(value_min, value_max)
        if(not((a in value_not_in[:,0]) and (b in value_not_in[:,1]))):
            c = (a,b)
            random_point_array.append(c)
    random_point_array = np.array(random_point_array)
    return random_point_array

'''change array to 2D array'''
def get_2D_array(origin_array,column_cut_len,row_cut_len):    
    array_2D = []
    origin_array_row = []
    origin_array_column = []
    origin_array_column = np.zeros(column_cut_len*row_cut_len*3+1)
    for i in range(len(origin_array)):
        origin_array_row = np.zeros(1)
        
        for j in range(len(origin_array[0])):
            for k in range(len(origin_array[0][0])):
                for l in range(len(origin_array[0][0][0])):
                    origin_array_row = np.column_stack((origin_array_row,origin_array[i][j][k][l]))
       
        origin_array_column = np.row_stack((origin_array_column,origin_array_row))
    array_2D = np.delete(origin_array_column,0,axis = 0)
    array_2D = np.delete(array_2D,0,axis = 1)    
    return array_2D

'''change prediction data to color data'''
def rebuild_color(predictions,valid,column_cut_len,row_cut_len):
    drawredcar = []
    drawground = []
    for i in range(len(predictions)):
        if predictions[i] == '1':
            drawredcar.append(valid[i].reshape(column_cut_len,row_cut_len,3))
        elif predictions[i] == '2':
            drawground.append(valid[i].reshape(column_cut_len,row_cut_len,3))
    drawredcar = np.array(drawredcar)        
    drawredcar = drawredcar.astype(int)
    drawground = np.array(drawground)        
    drawground = drawground.astype(int)
    return drawredcar,drawground

'''find red car position'''
def find_red_car(final_data,sample_position_array,sample_x_len,sample_y_len):
    position_array = []
    for i in range(len(final_data)):
        if final_data[i] == '1':
            x = i%len(sample_position_array[0])
            y = i//len(sample_position_array[0])
            position = (5+x*sample_x_len,5+y*sample_y_len)
            position_array.append(position)
    position_array = np.array(position_array)
    return position_array

'''HSV red color boundary'''
'''
lower_red1 = np.array([156, 43, 46])
upper_red1 = np.array([180, 255, 255])
lower_red2 = np.array([0, 43, 46])
upper_red2 = np.array([10, 255, 255])
'''
def color_pass_filter(red_car_filter,position_array):
#    hsv_array = []
    position_filter = []
    for i in range(len(red_car_filter)):
        n=0
        for j in range(len(red_car_filter[0])):
            for k in range(len(red_car_filter[0][0])):
                hsv = colorsys.rgb_to_hsv(red_car_filter[i][j][k][0]/255,red_car_filter[i][j][k][1]/255,red_car_filter[i][j][k][2]/255)
    #            hsv_array.append(hsv)        
                if (
                ((hsv[0]>=156.0/180 and hsv[0]<=180.0/180) or (hsv[0]>=0.0/180 and hsv[0]<=10.0/180)) and
                (hsv[1]>=80.0/255 and hsv[1]<=255.0/255) and
                (hsv[2]>=80.0/255 and hsv[2]<=255.0/255)):
                    n = n+1
        if n > 10:
            position_filter.append(position_array[i])
    position_filter = np.array(position_filter)
    return position_filter
    
'''-----------------function finish----------------------'''

'''def test as a function'''
def test(test_image_cut,test_cut,k):
    '''Import DataSet'''
    data_train = np.load("data_train.npy")
    data_test = np.load("data_test.npy")
    ground_truth = np.load("ground_truth.npy")
    more_red_cars = np.load("more_red_cars.npy")
    ground_truth = np.delete(ground_truth,ground_truth.shape[1]-1,axis = 1)
    
    '''get more red car position'''
    more_red_car_position = []
    for i in range(len(more_red_cars)):
        
        x = (more_red_cars[i][0] + more_red_cars[i][2])//2
        y = (more_red_cars[i][1] + more_red_cars[i][3])//2
        position = (x,y)
        more_red_car_position.append(position)
    more_red_car_position = np.array(more_red_car_position)
    
    '''show origin image'''
    #plt.figure()
    #plt.imshow(data_train)
    #plt.figure()
    #plt.imshow(data_test)
    
    '''save origin image'''
    #plt.imsave('data_train.jpg',data_train)
    plt.imsave('data_test.jpg',data_test)
    
    '''show the train figure'''
    #plt.figure()
    #plt.imshow(data_train)
    #for i in range(len(ground_truth)):
    #    plt.scatter(ground_truth[i][0],ground_truth[i][1], s=5, color='r')
    #plt.show()
    
    '''show red car'''
    #show_red_car('data_train.jpg',ground_truth)
    #show_red_car('data_train.jpg',more_red_car_position)
    
    '''get cut image'''
    #train_image_cut = cut_image(data_train,500,1500,4000,5000,'train_cut.jpg')
    #test_image_cut = cut_image(data_test,1000,1500,2000,2500,'test_cut.jpg')
    
    '''array spilt'''
    column_cut_len = 10
    row_cut_len = 10
    (sample_array,sample_position_array) = array_spilt(test_image_cut,column_cut_len,row_cut_len)
    
    '''get red car'''
    red_car = cut_figure(data_train,ground_truth[:,1],ground_truth[:,0])
    more_car = cut_figure(data_train,more_red_car_position[:,1],more_red_car_position[:,0])
    #for i in range(len(red_car)):
    #    plt.figure()
    #    plt.imshow(red_car[i])
    
    '''get random position'''
    value_not_in = np.row_stack((ground_truth,more_red_car_position))
    random_number = get_random_number(column_cut_len/2,len(data_train)-row_cut_len/2,100,value_not_in)
    
    '''get random data'''
    random_data = cut_figure(data_train,random_number[:,1],random_number[:,0])
    #for i in range(10):
    #    plt.figure()
    #    plt.imshow(random_data[i])
    
    '''add labels'''
    train_data_4D_labels = []
    train_data_4D = np.row_stack((red_car,more_car))
    train_data_4D = np.row_stack((train_data_4D,random_data))
    for i in range(len(red_car)):
        train_data_4D_labels.append('1')
    for k in range(len(more_car)):
        train_data_4D_labels.append('1')
    for j in range(len(random_data)):
        train_data_4D_labels.append('2')
    train_data_4D_labels = np.array(train_data_4D_labels)
    
    '''change data to 2D'''
    train_data_4D_2D = get_2D_array(train_data_4D,column_cut_len,row_cut_len)
    
    '''KNN test k'''
    X_train_4D, X_valid_4D, label_train_4D, label_valid_4D = train_test_split(train_data_4D_2D, train_data_4D_labels, test_size = 0.33, random_state = 20)
    
    X_train_2D = X_train_4D
    label_train_2D = label_train_4D
    X_valid_2D = X_valid_4D
    
    '''KNN train data'''
    test_M = k
    
    #using function of KNN
    knn = KNeighborsClassifier(test_M)
    #2D    
    # fitting the model
    knn.fit(X_train_2D, label_train_2D)
    
    # predict the response
    predictions_KNN_2D = knn.predict(X_valid_2D)
    
    
    '''change prediction data to color data'''
    (drawredcar, drawground) = rebuild_color(predictions_KNN_2D,X_valid_2D,column_cut_len,row_cut_len)
    
    #for i in range(5):
    #    plt.figure()
    #    plt.imshow(drawredcar[i])
    #for i in range(5):
    #    plt.figure()
    #    plt.imshow(drawground[i])
    #for i in range(5):
    #    plt.figure()
    #    plt.imshow(more_car[i])
      
    '''KNN test data'''
    data_test_2D = get_2D_array(sample_array,column_cut_len,row_cut_len)
    
    #choose k
    knn = KNeighborsClassifier(test_M)
    
    # fitting the model
    knn.fit(X_train_4D, label_train_4D)
    
    # predict the response
    test_KNN_HS = knn.predict(data_test_2D)
    
    '''find red car position'''
    position_array = find_red_car(test_KNN_HS,sample_position_array,column_cut_len,row_cut_len)
#    show_red_car(test_cut,position_array)
    
    
    '''color pass filter'''
    red_car_filter = cut_figure(test_image_cut,position_array[:,1],position_array[:,0])
    #for i in range(len(red_car_filter)):
    #    plt.figure()
    #    plt.imshow(red_car_filter[i])
    
    position_filter = color_pass_filter(red_car_filter,position_array)
    img = show_red_car(test_cut,position_filter)
    img.save('test_circle.jpg')    
        
    return position_filter

