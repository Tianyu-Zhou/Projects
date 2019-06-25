# -*- coding: utf-8 -*-
"""
Created on Tue Nov 27 17:10:31 2018

@author: 42238
"""
'''pytorch'''
import torch as pt
import torchvision as ptv
import numpy as np
import matplotlib.pyplot as plt
import torch.utils.data as Data
import math
from sklearn.decomposition import PCA
from sklearn.metrics import accuracy_score,confusion_matrix,classification_report
from sklearn.model_selection import train_test_split
from skimage import transform

pt.set_default_tensor_type('torch.DoubleTensor')
data_save_file_name = ('accuracy_logistics.npy')

class MLP(pt.nn.Module):
    def __init__(self):
        super(MLP,self).__init__()
        self.fc1 = pt.nn.Linear(3600,512)
        self.fc2 = pt.nn.Linear(512,128)
        self.fc3 = pt.nn.Linear(128,10)
        
    def forward(self,din):
        din = din.view(-1,60*60)
        dout = pt.nn.functional.relu(self.fc1(din))
        dout = pt.nn.functional.relu(self.fc2(dout))
        return pt.nn.functional.logsigmoid(self.fc3(dout))
model = MLP()
#print(model)

# optimizer and loss function
optimizer = pt.optim.SGD(model.parameters(),lr=0.1,momentum=0.9)
lossfunc = pt.nn.CrossEntropyLoss()

def AccuarcyCompute(pred,label):
    pred = pred.cpu().data.numpy()
    label = label.cpu().data.numpy()
#     print(pred.shape(),label.shape())
    test_np = (np.argmax(pred,1) == label)
    test_np = np.float32(test_np)
    return np.mean(test_np)

def list_average(num):
    nsum = 0
    for i in range(len(num)):
        nsum += num[i]
    return nsum / len(num)
    
'''find max matrix'''    
def find_max_mat(data):  
    size = []
    for i in range(len(data)):
        size.append(np.size(data[i]))
    max_size = np.max(size)
    position = np.argmax(size)
    mat_max_shape = np.shape(data[position])
    return mat_max_shape

def random_mat(number,shape):
    data_unknown = []
    labels_unknown = []
    for i in range(number):
        a = np.random.randint(0,2,size=shape)
        data_unknown.append(a)
        labels_unknown.append(-1)
    data_unknown = np.array(data_unknown)
    labels_unknown = np.array(labels_unknown)
    return data_unknown,labels_unknown

def ReLU_for_image(x):
    if (x<=6e-20): x = 0
    elif (x>6e-20): x = 1
    return x

data = np.load('ClassData.npy')
labels = np.load('ClassLabels.npy')

#max_shape = find_max_mat(data)
max_shape = (60,60)

'''get matrix data'''
data_mat_array = []
for i in range(len(data)):
    data_mat = transform.resize(data[i],(max_shape))
    data_mat_array.append(data_mat)

for i in range(len(data_mat_array)):
    for j in range(len(data_mat_array[i])):
        for k in range(len(data_mat_array[i][j])):
            data_mat_array[i][j][k] = ReLU_for_image(data_mat_array[i][j][k])
data_mat = np.array(data_mat_array)
#np.save('data_mat.npy',data_mat)

'''get 2D data'''
# =============================================================================
# max_size = max_shape[0]*max_shape[1]
# data_2D = np.zeros((len(data),max_size))
# data_2D_array = []
# for i in range(len(data)):
#     for j in range(len(data[i])):
#         for k in range(len(data[i][j])):
#             data_2D[i][j*len(data[i][j])+k] = data[i][j][k]
# data_2D = np.array(data_2D)
# #np.save('data_2D.npy',data_2D)
# =============================================================================

#data_2D = np.load('data_2D.npy')
#data_mat = np.load('data_mat.npy')

(data_unknown,labels_unknown) = random_mat(500,max_shape)

data_mat = np.concatenate((data_mat,data_unknown),axis=0)
labels = np.concatenate((labels,labels_unknown),axis=0)

for i in range(len(labels)):
    if labels[i] == -1: labels[i] = 0
    
data_train, data_valid, label_train, label_valid = train_test_split(data_mat, labels, test_size = 0.33, random_state = 20)

total_inputs = pt.from_numpy(data_train)
total_label_train = pt.from_numpy(label_train)

train_times = 200
'''batch'''
accuracy_array = []
for i in range(train_times):
    optimizer.zero_grad()
    
    total_inputs = pt.autograd.Variable(total_inputs)
    total_label_train = pt.autograd.Variable(total_label_train)

    total_outputs = model(total_inputs)

    loss = lossfunc(total_outputs,total_label_train)
    loss.backward()

    optimizer.step()

    print(i,":",AccuarcyCompute(total_outputs,total_label_train))
    accuracy = AccuarcyCompute(total_outputs,total_label_train)
    accuracy_array.append(accuracy)
accuracy_array = np.array(accuracy_array)
#np.save(data_save_file_name,accuracy_array)

outputs_pred = total_outputs.cpu().data.numpy()
label_train_pred = np.argmax(outputs_pred,1)
print (confusion_matrix(total_label_train,label_train_pred))
print ('train accuracy is :',accuracy_score(total_label_train,label_train_pred))

test_inputs = pt.from_numpy(data_valid)
test_labels = pt.from_numpy(label_valid)

test_inputs = pt.autograd.Variable(test_inputs)
test_labels = pt.autograd.Variable(test_labels)
test_output = model(test_inputs)

testdata_pred = test_output.cpu().data.numpy()
test_pred = np.argmax(testdata_pred,1)
print (confusion_matrix(test_labels,test_pred))
print ('test accuracy is :',accuracy_score(test_labels,test_pred))

train_times = np.arange(train_times)
'''draw image of accuracy'''
plt.figure()
plt.plot(train_times,accuracy_array)
plt.ylabel('accuracy')
plt.xlabel('train times')
plt.legend(['batch_accuracy'])
plt.title('accuracy probabilistic generative classifier for batch' )  

'''save model'''
pt.save(model.state_dict(),"mlp_params.pt")
pt.save(model,"mlp_model.pt")


