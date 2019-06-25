# -*- coding: utf-8 -*-
"""
Created on Fri Dec  7 12:23:22 2018

@author: 42238
"""
import torch as pt
import numpy as np
import sys
from skimage import transform

pt.set_default_tensor_type('torch.DoubleTensor')

if __name__ == '__main__':

    # To not call from command line, comment the following code block and use example below 
    # to use command line, call: python hw07.py K.jpg output

    if len(sys.argv) != 3 and len(sys.argv) != 4:
        print('usage: {} <in_filename> <out_filename> (--debug)'.format(sys.argv[0]))
        sys.exit(0)
    
    in_fname = sys.argv[1]
    out_fname = sys.argv[2]

    if len(sys.argv) == 4:
        debug = sys.argv[3] == '--debug'
    else:
        debug = False

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
model.load_state_dict(pt.load("mlp_params.pt"))

def ReLU_for_image(x):
    if (x<=6e-20): x = 0
    elif (x>6e-20): x = 1
    return x

data = np.load(in_fname)

'''Resize data'''
data_mat_array = []
for i in range(len(data)):
    data_mat = transform.resize(data[i],(60,60))
    data_mat_array.append(data_mat)

for i in range(len(data_mat_array)):
    for j in range(len(data_mat_array[i])):
        for k in range(len(data_mat_array[i][j])):
            data_mat_array[i][j][k] = ReLU_for_image(data_mat_array[i][j][k])

inputs = np.array(data_mat_array)
inputs = pt.from_numpy(inputs)
inputs = pt.autograd.Variable(inputs)
outputs = model(inputs)
outputs_pred = outputs.cpu().data.numpy()
label_pred = np.argmax(outputs_pred,1)
for i in range(len(label_pred)):
    if label_pred[i] == 0: label_pred[i] = -1
np.save(out_fname,label_pred)
