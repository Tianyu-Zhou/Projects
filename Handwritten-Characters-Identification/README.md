About the project
===================

This project has 2 python files: train.py and test.py, 2 data files: ClassData.npy and ClassLabels.npy, 2 model files: mlp_model.pt and mlp_params.pt

train.py
-------
train.py is used to train a model with ClassData.npy and ClassLabels.npy. We have already trained a model for you. The model and its parameters named mlp_model.pt and mlp_params.pt.

test.py
-------
test.py is used to test a data and get a output labels. test.py use the model we built from train.py. If you want to test some data, don't forget put test.py, mlp_model.pt and mlp_params.pt together.

The method of using test.py: python test.py input_file output_file (eg. python test.py inData.npy outlabel.npy)

Attention: test.py, mlp_model.pt and mlp_params.pt must put in one folder.
