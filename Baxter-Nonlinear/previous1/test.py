import rospy, baxter_interface, time, sys, os, math
import numpy as np
from std_msgs.msg import Empty, UInt16
from math import pi, cos, sin
import random
from sympy import *
from scipy.signal import butter, lfilter, freqz, freqs
import matplotlib.pyplot as plt

from baxter_core_msgs.msg import SEAJointState
from sensor_msgs.msg import JointState

import bax_controller
import trajectory

import time

D2R=pi/180.0;
R2D=1/D2R;

class Robot:
	def __init__(self):
		self.larm = baxter_interface.Limb("left")
		self.rarm = baxter_interface.Limb("right")
		self.larm.set_command_timeout(1)
		self.rarm.set_command_timeout(1)

def callbackL(data):
	global gCompL
	gCompL = np.array(data.gravity_model_effort)

def callbackR(data):
	global gCompR
	gCompR = np.array(data.gravity_model_effort)

def callefforts(data):
	global rightefforts
	rightefforts = np.array(data.effort)
	

rospy.init_node("qnda")
time.sleep(0.1)

pr = rospy.Publisher('robot/joint_state_publish_rate', UInt16, queue_size=1)
pr.publish(1000)
pcdl = rospy.Publisher('robot/limb/left/suppress_cuff_interaction', Empty, queue_size=1)
pcdr = rospy.Publisher('robot/limb/right/suppress_cuff_interaction', Empty, queue_size=1)
gdl = rospy.Publisher('/robot/limb/left/suppress_gravity_compensation', Empty, queue_size=1)
gdr = rospy.Publisher('/robot/limb/right/suppress_gravity_compensation', Empty, queue_size=1)
rospy.Subscriber('/robot/limb/left/gravity_compensation_torques', SEAJointState,callbackL)
rospy.Subscriber('/robot/limb/right/gravity_compensation_torques', SEAJointState,callbackR)
rospy.Subscriber('/robot/joint_states',JointState,callefforts)

#~ pcdl.publish()
#~ pcdr.publish()
r = rospy.Rate(1000)
bax = Robot()
		
larm_mapping = np.array(["left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"])
rarm_mapping = np.array(["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"])

def butter_lowpass(curoff, fs, order=5):
	nyq = 0.5 * fs
	normal_cutoff = cutoff / nyq
	b , a = butter(order, normal_cutoff, btype='lowpass', analog=False)
	return b , a 
	
def butter_lowpass_filter(data, cutoff, fs, order=5):
	b , a = butter_lowpass(cutoff, fs, order = order)
	y = lfilter(b, a, data)
	return y
	


# Gain
#~ #LEFT
#~ k11l = 10; k12l = 10; k13l = 1; k14l = 8; k15l = 1; k16l = 8; k17l = 1;
#~ k1l_array = np.array([k11l, k12l, k13l, k14l, k15l, k16l, k17l]);
#~ k1l = np.diag(k1l_array);
#~ 
#~ k21l = 1; k22l = 1; k23l = 1; k24l = 1.5; k25l = 0.5; k26l = 1; k27l = 1;
#~ k2l_array = np.array([k21l, k22l, k23l, k24l, k25l, k26l, k27l]);
#~ k2l = np.diag(k2l_array);
#~ 
#~ Be1l = 1;  Be2l = 1; Be3l = 1;  Be4l = 1; Be5l = 1; Be6l = 1; Be7l = 0.5;
#~ Bel_array = np.array([Be1l, Be2l, Be3l, Be4l, Be5l, Be6l, Be7l]);
#~ Bel = np.diag(Bel_array);
#~ 
#~ #RIGHT
#~ k11r = 10; k12r = 10; k13r = 1; k14r = 8; k15r = 1; k16r = 8; k17r = 1;
#~ k1r_array = np.array([k11r, k12r, k13r, k14r, k15r, k16r, k17r]);
#~ k1r = np.diag(k1r_array);
#~ 
#~ k21r = 1; k22r = 1; k23r = 1; k24r = 1.5; k25r = 0.5; k26r = 1; k27r = 1;
#~ k2r_array = np.array([k21r, k22r, k23r, k24r, k25r, k26r, k27r]);
#~ k2r = np.diag(k2r_array);
#~ 
#~ Be1r = 1;  Be2r = 1; Be3r = 1;  Be4r = 1; Be5r = 1; Be6r = 1; Be7r = 0.5;
#~ Ber_array = np.array([Be1r, Be2r, Be3r, Be4r, Be5r, Be6r, Be7r]);
#~ Ber = np.diag(Ber_array);




#LEFT
k11l = 1; k12l = 4; k13l = 2; k14l = 5; k15l = 2; k16l = 5; k17l = 1;
k1l_array = np.array([k11l, k12l, k13l, k14l, k15l, k16l, k17l]);
k1l = np.diag(k1l_array);

k21l = 12; k22l = 12; k23l = 3; k24l = 5; k25l = 2.5; k26l = 3; k27l = 3;
k2l_array = np.array([k21l, k22l, k23l, k24l, k25l, k26l, k27l]);
k2l = np.diag(k2l_array);

Be1l = 35;  Be2l = 6; Be3l = 10;  Be4l = 15; Be5l = 8;  Be6l = 15; Be7l = 10;
Bel_array = np.array([Be1l, Be2l, Be3l, Be4l, Be5l, Be6l, Be7l]);
Bel = np.diag(Bel_array);

#RIGHT
k11r = 1; k12r = 4; k13r = 1; k14r = 5; k15r = 2; k16r = 5; k17r = 0.5;
k1r_array = np.array([k11r, k12r, k13r, k14r, k15r, k16r, k17r]);
k1r = np.diag(k1r_array);

k21r = 12; k22r = 12; k23r = 5; k24r = 5; k25r = 3; k26r = 3; k27r = 3;
k2r_array = np.array([k21r, k22r, k23r, k24r, k25r, k26r, k27r]);
k2r = np.diag(k2r_array);

Be1r = 35;  Be2r = 6; Be3r = 10;  Be4r = 15; Be5r = 6;  Be6r = 15; Be7r = 10;
Ber_array = np.array([Be1r, Be2r, Be3r, Be4r, Be5r, Be6r, Be7r]);
Ber = np.diag(Ber_array);




pos = 0
vel = 0
acc = 0
ql_d = np.array([-0.75, 0, 0, 0, 0, 0, 0])
qdotl_d = np.array([vel, vel, vel, vel, vel, vel, vel])
qddotl_d = np.array([acc, acc, acc, acc, acc, acc, acc])
pos = 0
vel = 0
acc = 0
qr_d = np.array([0.75, 0, 0, 0, 0, 0, 0])
qdotr_d = np.array([vel, vel, vel, vel, vel, vel, vel])
qddotr_d = np.array([acc, acc, acc, acc, acc, acc, acc])

bax.larm.move_to_joint_positions(dict(zip(larm_mapping, ql_d)))	
#~ bax.rarm.move_to_joint_positions(dict(zip(rarm_mapping, qr_d)))	

while not rospy.is_shutdown():
	
	#~ gdl.publish()
	#~ gdr.publish()
	print(time.time())
	print(rospy.get_time())
	
	qr = np.array(np.vectorize(bax.rarm.joint_angles().get)(rarm_mapping))
	qdotr = np.array(np.vectorize(bax.rarm.joint_velocities().get)(rarm_mapping))
	ql = np.array(np.vectorize(bax.larm.joint_angles().get)(larm_mapping))
	qdotl = np.array(np.vectorize(bax.larm.joint_velocities().get)(larm_mapping))
	ql_d = np.array([-0.75, 0, 0, 0, 0, 0, 0])
	qr_d = np.array([0.75, -0.3, 0, 0, 0, 0, 0])	
	#~ print(gCompR)
	#~ print(gCompL)
	
	#~ psir = np.zeros(7)
	#~ psil = np.zeros(7)
	psir = bax_controller.get_torque(qr, qdotr, qddotr_d, qr_d, qdotr_d, k1r, k2r, Ber)
	psil = bax_controller.get_torque(ql, qdotl, qddotl_d, ql_d, qdotl_d, k1l, k2l, Bel)
	R = dict(zip(rarm_mapping, (psir)))
	L = dict(zip(larm_mapping, (psil)))
	#~ bax.rarm.set_joint_torques(R)
	bax.larm.set_joint_torques(L)
	
	
	el = ql_d - ql;
	er = qr_d - qr;
	
	print('LEFT:')
	print(psil)
	print(el)
	print(abs(el)<=0.03)
	#~ print('RIGHT:')
	#~ print(psir)
	#~ print(er)
	#~ print(abs(er)<=0.03)
	
	#~ print(rightefforts)
	
	#~ rr = np.array(np.vectorize(bax.larm.joint_efforts().get)(larm_mapping))
	#~ print(rr)



#~ qr_d = np.array([0.75, 0, 0, 0, 0, 0, 0])
#~ ql_d = np.array([-0.75, 0, 0, 0, 0, 0, 0])
#~ bax.larm.move_to_joint_positions(dict(zip(larm_mapping, ql_d)))	
#~ bax.rarm.move_to_joint_positions(dict(zip(rarm_mapping, qr_d)))	
#~ thl = np.array(np.vectorize(bax.larm.joint_angles().get)(larm_mapping))
#~ thr = np.array(np.vectorize(bax.rarm.joint_angles().get)(rarm_mapping))

#~ qr_d = np.array([0.75, 0.8, -3, 0.6, 1.5, 0, 0])
#~ ql_d = np.array([-0.75, 0.8, 3, 1.2, -1.5, 0, 0])
#~ bax.larm.move_to_joint_positions(dict(zip(larm_mapping, ql_d)))	
#~ bax.rarm.move_to_joint_positions(dict(zip(rarm_mapping, qr_d)))	
#~ thl = np.array(np.vectorize(bax.larm.joint_angles().get)(larm_mapping))
#~ thr = np.array(np.vectorize(bax.rarm.joint_angles().get)(rarm_mapping))


#~ qr_d = np.array([0.65, 0.8, -3, 0.6, 0, 0, 1.5])
#~ ql_d = np.array([-0.65, 0.8, 3, 1.2, 0, 0, -1.5])
#~ bax.larm.move_to_joint_positions(dict(zip(larm_mapping, ql_d)))	
#~ bax.rarm.move_to_joint_positions(dict(zip(rarm_mapping, qr_d)))	
#~ thl = np.array(np.vectorize(bax.larm.joint_angles().get)(larm_mapping))
#~ thr = np.array(np.vectorize(bax.rarm.joint_angles().get)(rarm_mapping))
