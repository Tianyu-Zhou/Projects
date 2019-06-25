How to use run code
===================

This project has 3 python files: train.py test.py run.py

Using these python files with: data_test.npy	data_train.npy	ground_truth.npy	more_red_cars.npy

Attention: there is a more_red_cars.npy file for more cars, do not forget it.

run.py
-----
This is a file to run this project, just run run.py directly. It return the best k value for KNN, one image for data_train, one image for date_test, one red_car_pasition_train data corresponds to train_cut.jpg, and one red_car_position_test corresponds to test_cut.jpg.

train.py
-------
Do NOT run train.py or test.py, all the content inside it are functions. This file is used to train the best k. After that use k to find red car. The meaning of all function are described in report.pdf.

test.py
-------
Do NOT run train.py or test.py, all the content inside it are functions. This file is to test if train.py works well or not. It will create a image to show where is the red car.
