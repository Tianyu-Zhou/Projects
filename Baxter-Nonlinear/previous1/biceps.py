import rospy, baxter_interface, time, sys, os, math
import numpy as np
from std_msgs.msg import Empty, UInt16
from math import pi, cos, sin
import random
from sympy import *


import bax_controller
import trajectory

D2R=pi/180.0;
R2D=1/D2R;

class Robot:
	def __init__(self):
		self.larm = baxter_interface.Limb("left")
		self.rarm = baxter_interface.Limb("right")
		self.larm.set_command_timeout(1)
		self.rarm.set_command_timeout(1)

rospy.init_node("qnda")
time.sleep(0.1)

bax = Robot()
		
larm_mapping = np.array(["left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"])
rarm_mapping = np.array(["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"])

a = input('Input 1 to initialize, other number to continue')
if a == 1:
	qr_d = np.array([0.75, 0, 0, 0, 0, 0, 0])
	ql_d = np.array([-0.75, 0, 0, 0, 0, 0, 0])
	bax.larm.move_to_joint_positions(dict(zip(larm_mapping, ql_d)))	
	bax.rarm.move_to_joint_positions(dict(zip(rarm_mapping, qr_d)))	

	qr_d = np.array([0.75, 1, -3, 0.6, 1.5, 0, 0])
	ql_d = np.array([-0.75, 1, 3, 1.5, -1.5, 0, 0])
	bax.larm.move_to_joint_positions(dict(zip(larm_mapping, ql_d)))	
	bax.rarm.move_to_joint_positions(dict(zip(rarm_mapping, qr_d)))	

input('Input any number')
print('Get first data')
thl_1 = np.array(np.vectorize(bax.larm.joint_angles().get)(larm_mapping))
thr_1 = np.array(np.vectorize(bax.rarm.joint_angles().get)(rarm_mapping))

input('Input any number')
print('Get second data')
thl_2 = np.array(np.vectorize(bax.larm.joint_angles().get)(larm_mapping))
thr_2 = np.array(np.vectorize(bax.rarm.joint_angles().get)(rarm_mapping))

input('Input any number')

print(thl_1)
print(thl_2)
print(thr_1)
print(thr_2)

t_d = 3
(trajl1_0, trajl1_dot_0, trajl1_2dot_0) = trajectory.get_1trajectory(thl_1[0], thl_2[0], t_d)
(trajl1_1, trajl1_dot_1, trajl1_2dot_1) = trajectory.get_1trajectory(thl_1[1], thl_2[1], t_d)
(trajl1_2, trajl1_dot_2, trajl1_2dot_2) = trajectory.get_1trajectory(thl_1[2], thl_2[2], t_d)
(trajl1_3, trajl1_dot_3, trajl1_2dot_3) = trajectory.get_1trajectory(thl_1[3], thl_2[3], t_d)
(trajl1_4, trajl1_dot_4, trajl1_2dot_4) = trajectory.get_1trajectory(thl_1[4], thl_2[4], t_d)
(trajl1_5, trajl1_dot_5, trajl1_2dot_5) = trajectory.get_1trajectory(thl_1[5], thl_2[5], t_d)
(trajl1_6, trajl1_dot_6, trajl1_2dot_6) = trajectory.get_1trajectory(thl_1[6], thl_2[6], t_d)

(trajr1_0, trajr1_dot_0, trajr1_2dot_0) = trajectory.get_1trajectory(thr_1[0], thr_2[0], t_d)
(trajr1_1, trajr1_dot_1, trajr1_2dot_1) = trajectory.get_1trajectory(thr_1[1], thr_2[1], t_d)
(trajr1_2, trajr1_dot_2, trajr1_2dot_2) = trajectory.get_1trajectory(thr_1[2], thr_2[2], t_d)
(trajr1_3, trajr1_dot_3, trajr1_2dot_3) = trajectory.get_1trajectory(thr_1[3], thr_2[3], t_d)
(trajr1_4, trajr1_dot_4, trajr1_2dot_4) = trajectory.get_1trajectory(thr_1[4], thr_2[4], t_d)
(trajr1_5, trajr1_dot_5, trajr1_2dot_5) = trajectory.get_1trajectory(thr_1[5], thr_2[5], t_d)
(trajr1_6, trajr1_dot_6, trajr1_2dot_6) = trajectory.get_1trajectory(thr_1[6], thr_2[6], t_d)

thl_init = np.array(np.vectorize(bax.larm.joint_angles().get)(larm_mapping))
thr_init = np.array(np.vectorize(bax.rarm.joint_angles().get)(rarm_mapping))

(trajl_0, trajl_dot_0, trajl_2dot_0) = trajectory.get_1trajectory(thl_init[0], thl_2[0], t_d)
(trajl_1, trajl_dot_1, trajl_2dot_1) = trajectory.get_1trajectory(thl_init[1], thl_2[1], t_d)
(trajl_2, trajl_dot_2, trajl_2dot_2) = trajectory.get_1trajectory(thl_init[2], thl_2[2], t_d)
(trajl_3, trajl_dot_3, trajl_2dot_3) = trajectory.get_1trajectory(thl_init[3], thl_2[3], t_d)
(trajl_4, trajl_dot_4, trajl_2dot_4) = trajectory.get_1trajectory(thl_init[4], thl_2[4], t_d)
(trajl_5, trajl_dot_5, trajl_2dot_5) = trajectory.get_1trajectory(thl_init[5], thl_2[5], t_d)
(trajl_6, trajl_dot_6, trajl_2dot_6) = trajectory.get_1trajectory(thl_init[6], thl_2[6], t_d)

(trajr_0, trajr_dot_0, trajr_2dot_0) = trajectory.get_1trajectory(thr_init[0], thr_2[0], t_d)
(trajr_1, trajr_dot_1, trajr_2dot_1) = trajectory.get_1trajectory(thr_init[1], thr_2[1], t_d)
(trajr_2, trajr_dot_2, trajr_2dot_2) = trajectory.get_1trajectory(thr_init[2], thr_2[2], t_d)
(trajr_3, trajr_dot_3, trajr_2dot_3) = trajectory.get_1trajectory(thr_init[3], thr_2[3], t_d)
(trajr_4, trajr_dot_4, trajr_2dot_4) = trajectory.get_1trajectory(thr_init[4], thr_2[4], t_d)
(trajr_5, trajr_dot_5, trajr_2dot_5) = trajectory.get_1trajectory(thr_init[5], thr_2[5], t_d)
(trajr_6, trajr_dot_6, trajr_2dot_6) = trajectory.get_1trajectory(thr_init[6], thr_2[6], t_d)



# Gain
#LEFT
k11l = 1; k12l = 4; k13l = 2; k14l = 5; k15l = 2; k16l = 4; k17l = 1;
k1l_array = np.array([k11l, k12l, k13l, k14l, k15l, k16l, k17l]);
k1l = np.diag(k1l_array);

k21l = 12; k22l = 12; k23l = 3; k24l = 5; k25l = 2.5; k26l = 2; k27l = 3;
k2l_array = np.array([k21l, k22l, k23l, k24l, k25l, k26l, k27l]);
k2l = np.diag(k2l_array);

Be1l = 35;  Be2l = 6; Be3l = 10;  Be4l = 15; Be5l = 8;  Be6l = 15; Be7l = 10;
Bel_array = np.array([Be1l, Be2l, Be3l, Be4l, Be5l, Be6l, Be7l]);
Bel = np.diag(Bel_array);

#RIGHT
k11r = 1; k12r = 4; k13r = 1; k14r = 5; k15r = 2; k16r = 4; k17r = 0.5;
k1r_array = np.array([k11r, k12r, k13r, k14r, k15r, k16r, k17r]);
k1r = np.diag(k1r_array);

k21r = 12; k22r = 12; k23r = 5; k24r = 5; k25r = 3; k26r = 2; k27r = 3;
k2r_array = np.array([k21r, k22r, k23r, k24r, k25r, k26r, k27r]);
k2r = np.diag(k2r_array);

Be1r = 35;  Be2r = 6; Be3r = 10;  Be4r = 15; Be5r = 6;  Be6r = 15; Be7r = 10;
Ber_array = np.array([Be1r, Be2r, Be3r, Be4r, Be5r, Be6r, Be7r]);
Ber = np.diag(Ber_array);

n = -1

time_init = rospy.get_time()
zero = np.zeros(7)

while not rospy.is_shutdown():
	cur_time = rospy.get_time()	
		
	if n == -2:
		time_init = rospy.get_time()
		n = 0
	
	#~ if n == -3:
		#~ t = cur_time - time_init
		#~ qr = np.array(np.vectorize(bax.rarm.joint_angles().get)(rarm_mapping))
		#~ qdotr = np.array(np.vectorize(bax.rarm.joint_velocities().get)(rarm_mapping))
		#~ ql = np.array(np.vectorize(bax.larm.joint_angles().get)(larm_mapping))
		#~ qdotl = np.array(np.vectorize(bax.larm.joint_velocities().get)(larm_mapping))
		#~ 
		#~ psir = bax_controller.get_torque(qr, qdotr, zero, thr_init, zero, k1r, k2r, Ber)
		#~ psil = bax_controller.get_torque(ql, qdotl, zero, thl_init, zero, k1l, k2l, Bel)
		#~ R = dict(zip(rarm_mapping, (psir)))
		#~ L = dict(zip(larm_mapping, (psil)))
		#~ bax.rarm.set_joint_torques(R)
		#~ bax.larm.set_joint_torques(L)
		#~ if abs(t-2)<=0.01:
			#~ n = -1
	
	if n == -1:
		t = cur_time - time_init
		print(t)
		qr_d = np.array([trajr_0(t), trajr_1(t), trajr_2(t), trajr_3(t), trajr_4(t), trajr_5(t), trajr_6(t)])
		qdotr_d = np.array([trajr_dot_0(t), trajr_dot_1(t), trajr_dot_2(t), trajr_dot_3(t), trajr_dot_4(t), trajr_dot_5(t), trajr_dot_6(t)])
		qddotr_d = np.array([trajr_2dot_0(t), trajr_2dot_1(t), trajr_2dot_2(t), trajr_2dot_3(t), trajr_2dot_4(t), trajr_2dot_5(t), trajr_2dot_6(t)])	
		
		ql_d = np.array([trajl_0(t), trajl_1(t), trajl_2(t), trajl_3(t), trajl_4(t), trajl_5(t), trajl_6(t)])
		qdotl_d = np.array([trajl_dot_0(t), trajl_dot_1(t), trajl_dot_2(t), trajl_dot_3(t), trajl_dot_4(t), trajl_dot_5(t), trajl_dot_6(t)])
		qddotl_d = np.array([trajl_2dot_0(t), trajl_2dot_1(t), trajl_2dot_2(t), trajl_2dot_3(t), trajl_2dot_4(t), trajl_2dot_5(t), trajl_2dot_6(t)])	
		
		qr = np.array(np.vectorize(bax.rarm.joint_angles().get)(rarm_mapping))
		qdotr = np.array(np.vectorize(bax.rarm.joint_velocities().get)(rarm_mapping))		
		ql = np.array(np.vectorize(bax.larm.joint_angles().get)(larm_mapping))
		qdotl = np.array(np.vectorize(bax.larm.joint_velocities().get)(larm_mapping))		
		
		psir = bax_controller.get_torque(qr, qdotr, qddotr_d, qr_d, qdotr_d, k1r, k2r, Ber)	
		psil = bax_controller.get_torque(ql, qdotl, qddotl_d, ql_d, qdotl_d, k1l, k2l, Bel)	
		R = dict(zip(rarm_mapping, (psir)))
		L = dict(zip(larm_mapping, (psil)))
		bax.rarm.set_joint_torques(R)
		bax.larm.set_joint_torques(L)
		if abs(t-t_d)<=0.01:
			n = -2
			
	if n == 0:
		t = t_d - (cur_time - time_init)
		print(t)
		#~ print(cur_time)
		#~ print(time_init)
		qr_d = np.array([trajr1_0(t), trajr1_1(t), trajr1_2(t), trajr1_3(t), trajr1_4(t), trajr1_5(t), trajr1_6(t)])
		qdotr_d = np.array([trajr1_dot_0(t), trajr1_dot_1(t), trajr1_dot_2(t), trajr1_dot_3(t), trajr1_dot_4(t), trajr1_dot_5(t), trajr1_dot_6(t)])
		qddotr_d = np.array([trajr1_2dot_0(t), trajr1_2dot_1(t), trajr1_2dot_2(t), trajr1_2dot_3(t), trajr1_2dot_4(t), trajr1_2dot_5(t), trajr1_2dot_6(t)])	
		
		ql_d = np.array([trajl1_0(t), trajl1_1(t), trajl1_2(t), trajl1_3(t), trajl1_4(t), trajl1_5(t), trajl1_6(t)])
		qdotl_d = np.array([trajl1_dot_0(t), trajl1_dot_1(t), trajl1_dot_2(t), trajl1_dot_3(t), trajl1_dot_4(t), trajl1_dot_5(t), trajl1_dot_6(t)])
		qddotl_d = np.array([trajl1_2dot_0(t), trajl1_2dot_1(t), trajl1_2dot_2(t), trajl1_2dot_3(t), trajl1_2dot_4(t), trajl1_2dot_5(t), trajl1_2dot_6(t)])	
		
		qr = np.array(np.vectorize(bax.rarm.joint_angles().get)(rarm_mapping))
		qdotr = np.array(np.vectorize(bax.rarm.joint_velocities().get)(rarm_mapping))		
		ql = np.array(np.vectorize(bax.larm.joint_angles().get)(larm_mapping))
		qdotl = np.array(np.vectorize(bax.larm.joint_velocities().get)(larm_mapping))		
		
		psir = bax_controller.get_torque(qr, qdotr, qddotr_d, qr_d, qdotr_d, k1r, k2r, Ber)	
		psil = bax_controller.get_torque(ql, qdotl, qddotl_d, ql_d, qdotl_d, k1l, k2l, Bel)	
		R = dict(zip(rarm_mapping, (psir)))
		L = dict(zip(larm_mapping, (psil)))
		bax.rarm.set_joint_torques(R)
		bax.larm.set_joint_torques(L)
		
		#~ print(qr)
		#~ print(thr_1)
		#~ print('\n')
		#~ print(ql)
		#~ print(thl_1)
		#~ print(abs(qr-thr_1)<=0.3)
		#~ print(abs(qr-thr_1))
		#~ print((abs(qr-thr_1)<=0.3).all())
		#~ if ((abs(qr-thr_1)<=0.3).all()) or abs(t-t_d)<=0.01:
			#~ n = 1
		if abs(t-0)<=0.01:
			n = 1
	if n == 1:
		t = (cur_time - time_init - t_d)
		print(t)
		#~ print(cur_time)
		#~ print(time_init)
		qr_d = np.array([trajr1_0(t), trajr1_1(t), trajr1_2(t), trajr1_3(t), trajr1_4(t), trajr1_5(t), trajr1_6(t)])
		qdotr_d = np.array([trajr1_dot_0(t), trajr1_dot_1(t), trajr1_dot_2(t), trajr1_dot_3(t), trajr1_dot_4(t), trajr1_dot_5(t), trajr1_dot_6(t)])
		qddotr_d = np.array([trajr1_2dot_0(t), trajr1_2dot_1(t), trajr1_2dot_2(t), trajr1_2dot_3(t), trajr1_2dot_4(t), trajr1_2dot_5(t), trajr1_2dot_6(t)])	
		
		ql_d = np.array([trajl1_0(t), trajl1_1(t), trajl1_2(t), trajl1_3(t), trajl1_4(t), trajl1_5(t), trajl1_6(t)])
		qdotl_d = np.array([trajl1_dot_0(t), trajl1_dot_1(t), trajl1_dot_2(t), trajl1_dot_3(t), trajl1_dot_4(t), trajl1_dot_5(t), trajl1_dot_6(t)])
		qddotl_d = np.array([trajl1_2dot_0(t), trajl1_2dot_1(t), trajl1_2dot_2(t), trajl1_2dot_3(t), trajl1_2dot_4(t), trajl1_2dot_5(t), trajl1_2dot_6(t)])	
		
		qr = np.array(np.vectorize(bax.rarm.joint_angles().get)(rarm_mapping))
		qdotr = np.array(np.vectorize(bax.rarm.joint_velocities().get)(rarm_mapping))		
		ql = np.array(np.vectorize(bax.larm.joint_angles().get)(larm_mapping))
		qdotl = np.array(np.vectorize(bax.larm.joint_velocities().get)(larm_mapping))		
		
		psir = bax_controller.get_torque(qr, qdotr, qddotr_d, qr_d, qdotr_d, k1r, k2r, Ber)	
		psil = bax_controller.get_torque(ql, qdotl, qddotl_d, ql_d, qdotl_d, k1l, k2l, Bel)	
		R = dict(zip(rarm_mapping, (psir)))
		L = dict(zip(larm_mapping, (psil)))
		bax.rarm.set_joint_torques(R)
		bax.larm.set_joint_torques(L)
		
		#~ print(abs(qr-thr_2)<=0.3)
		#~ print(abs(qr-thr_2))
		#~ print((abs(qr-thr_2)<=0.3).all())
		#~ if ((abs(qr-thr_2)<=0.3).all()) or abs(t-t_d)<=0.01:
			#~ n = 2
		if abs(t-t_d)<=0.01:
			n = -2


		
	#~ if n == 2:
		#~ qr_d = np.array([0.75, 0, 0, 0, 0, 0, 0])
		#~ ql_d = np.array([-0.75, 0, 0, 0, 0, 0, 0])
		#~ bax.larm.move_to_joint_positions(dict(zip(larm_mapping, ql_d)))	
		#~ bax.rarm.move_to_joint_positions(dict(zip(rarm_mapping, qr_d)))	
		#~ thl = np.array(np.vectorize(bax.larm.joint_angles().get)(larm_mapping))
		#~ thr = np.array(np.vectorize(bax.rarm.joint_angles().get)(rarm_mapping))
#~ 
		#~ if ((abs(thr-qr_d)<=0.1).all) or t >5:
			#~ break
