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

thl_init = np.array(np.vectorize(bax.larm.joint_angles().get)(larm_mapping))
thr_init = np.array(np.vectorize(bax.rarm.joint_angles().get)(rarm_mapping))

pos1 = np.array([0.3079, -0.3133, 0.0675, 0.7352, -0.0410, 1.0753, -0.4057])
pos2 = np.array([0.5361, -0.6228, -0.0161, 0.8046, -0.1308, -0.2155, 0.1568])
pos3 = np.array([1.2429, -0.6845, -0.0940, 1.0339, 0.1246, -0.3003, -0.0031])
pos4 = np.array([1.3484, -0.2589, -0.1373, 0.7267, 0.1538, 1.0205, 0.4989])
#~ print(thl_init)
#~ print(thr_init)
#~ print(abs(thr_init-pos1))
#~ print(abs(thr_init-pos1)<=0.1)
#~ print((abs(thr_init-pos1)<=0.1).all())

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

t_d = 5

n = 0
(traj1_0, traj1_dot_0, traj1_2dot_0) = trajectory.get_1trajectory(thr_init[0], pos1[0], t_d)
(traj1_1, traj1_dot_1, traj1_2dot_1) = trajectory.get_1trajectory(thr_init[1], pos1[1], t_d)
(traj1_2, traj1_dot_2, traj1_2dot_2) = trajectory.get_1trajectory(thr_init[2], pos1[2], t_d)
(traj1_3, traj1_dot_3, traj1_2dot_3) = trajectory.get_1trajectory(thr_init[3], pos1[3], t_d)
(traj1_4, traj1_dot_4, traj1_2dot_4) = trajectory.get_1trajectory(thr_init[4], pos1[4], t_d)
(traj1_5, traj1_dot_5, traj1_2dot_5) = trajectory.get_1trajectory(thr_init[5], pos1[5], t_d)
(traj1_6, traj1_dot_6, traj1_2dot_6) = trajectory.get_1trajectory(thr_init[6], pos1[6], t_d)

(traj3_10, traj3_20, traj3_30, traj3_dot_10, traj3_dot_20, traj3_dot_30, traj3_2dot_10, traj3_2dot_20, traj3_2dot_30) = trajectory.get_3trajectory(pos1[0], pos2[0], pos3[0], pos4[0], t_d)
(traj3_11, traj3_21, traj3_31, traj3_dot_11, traj3_dot_21, traj3_dot_31, traj3_2dot_11, traj3_2dot_21, traj3_2dot_31) = trajectory.get_3trajectory(pos1[1], pos2[1], pos3[1], pos4[1], t_d)
(traj3_12, traj3_22, traj3_32, traj3_dot_12, traj3_dot_22, traj3_dot_32, traj3_2dot_12, traj3_2dot_22, traj3_2dot_32) = trajectory.get_3trajectory(pos1[2], pos2[2], pos3[2], pos4[2], t_d)
(traj3_13, traj3_23, traj3_33, traj3_dot_13, traj3_dot_23, traj3_dot_33, traj3_2dot_13, traj3_2dot_23, traj3_2dot_33) = trajectory.get_3trajectory(pos1[3], pos2[3], pos3[3], pos4[3], t_d)
(traj3_14, traj3_24, traj3_34, traj3_dot_14, traj3_dot_24, traj3_dot_34, traj3_2dot_14, traj3_2dot_24, traj3_2dot_34) = trajectory.get_3trajectory(pos1[4], pos2[4], pos3[4], pos4[4], t_d)
(traj3_15, traj3_25, traj3_35, traj3_dot_15, traj3_dot_25, traj3_dot_35, traj3_2dot_15, traj3_2dot_25, traj3_2dot_35) = trajectory.get_3trajectory(pos1[5], pos2[5], pos3[5], pos4[5], t_d)
(traj3_16, traj3_26, traj3_36, traj3_dot_16, traj3_dot_26, traj3_dot_36, traj3_2dot_16, traj3_2dot_26, traj3_2dot_36) = trajectory.get_3trajectory(pos1[6], pos2[6], pos3[6], pos4[6], t_d)


n = 0

time_init = rospy.get_time()
while not rospy.is_shutdown():
	cur_time = rospy.get_time()
	if n == 0:
		t = cur_time - time_init
		print(t)
		qr_d = np.array([traj1_0(t), traj1_1(t), traj1_2(t), traj1_3(t), traj1_4(t), traj1_5(t), traj1_6(t)])
		qdotr_d = np.array([traj1_dot_0(t), traj1_dot_1(t), traj1_dot_2(t), traj1_dot_3(t), traj1_dot_4(t), traj1_dot_5(t), traj1_dot_6(t)])
		qddotr_d = np.array([traj1_2dot_0(t), traj1_2dot_1(t), traj1_2dot_2(t), traj1_2dot_3(t), traj1_2dot_4(t), traj1_2dot_5(t), traj1_2dot_6(t)])	
		
		qr = np.array(np.vectorize(bax.rarm.joint_angles().get)(rarm_mapping))
		qdotr = np.array(np.vectorize(bax.rarm.joint_velocities().get)(rarm_mapping))		
		
		psir = bax_controller.get_torque(qr, qdotr, qddotr_d, qr_d, qdotr_d, k1r, k2r, Ber)	
		R = dict(zip(rarm_mapping, (psir)))
		bax.rarm.set_joint_torques(R)
		
		#~ print(abs(qr-pos1))
		#~ print((abs(qr-pos1)<=0.3).all())
		#~ if ((abs(qr-pos1)<=0.05).all()) or abs(t-t_d)<=0.01:
		if abs(t-t_d)<=0.01:
			n = 1
						
	if n == 1:
		t = cur_time - time_init - n*t_d
		print(t)
		qr_d = np.array([traj3_10(t), traj3_11(t), traj3_12(t), traj3_13(t), traj3_14(t), traj3_15(t), traj3_16(t)])
		qdotr_d = np.array([traj3_dot_10(t), traj3_dot_11(t), traj3_dot_12(t), traj3_dot_13(t), traj3_dot_14(t), traj3_dot_15(t), traj3_dot_16(t)])
		qddotr_d = np.array([traj3_2dot_10(t), traj3_2dot_11(t), traj3_2dot_12(t), traj3_2dot_13(t), traj3_2dot_14(t), traj3_2dot_15(t), traj3_2dot_16(t)])
		
		qr = np.array(np.vectorize(bax.rarm.joint_angles().get)(rarm_mapping))
		qdotr = np.array(np.vectorize(bax.rarm.joint_velocities().get)(rarm_mapping))		
		
		psir = bax_controller.get_torque(qr, qdotr, qddotr_d, qr_d, qdotr_d, k1r, k2r, Ber)	
		R = dict(zip(rarm_mapping, (psir)))
		bax.rarm.set_joint_torques(R)

		#~ print(abs(qr-pos2))
		#~ print((abs(qr-pos2)<=0.3).all())
		#~ if ((abs(qr-pos2)<=0.05).all()) or abs(t-t_d)<=0.01:
		if abs(t-t_d)<=0.01:
			n = 2
						
	if n == 2:
		t = cur_time - time_init - n*t_d
		print(t)
		qr_d = np.array([traj3_20(t), traj3_21(t), traj3_22(t), traj3_23(t), traj3_24(t), traj3_25(t), traj3_26(t)])
		qdotr_d = np.array([traj3_dot_20(t), traj3_dot_21(t), traj3_dot_22(t), traj3_dot_23(t), traj3_dot_24(t), traj3_dot_25(t), traj3_dot_26(t)])
		qddotr_d = np.array([traj3_2dot_20(t), traj3_2dot_21(t), traj3_2dot_22(t), traj3_2dot_23(t), traj3_2dot_24(t), traj3_2dot_25(t), traj3_2dot_26(t)])
		
		qr = np.array(np.vectorize(bax.rarm.joint_angles().get)(rarm_mapping))
		qdotr = np.array(np.vectorize(bax.rarm.joint_velocities().get)(rarm_mapping))		
		
		psir = bax_controller.get_torque(qr, qdotr, qddotr_d, qr_d, qdotr_d, k1r, k2r, Ber)	
		R = dict(zip(rarm_mapping, (psir)))
		bax.rarm.set_joint_torques(R)

		#~ print(abs(qr-pos3))
		#~ print((abs(qr-pos3)<=0.3).all())
		#~ if ((abs(qr-pos3)<=0.05).all()) or abs(t-t_d)<=0.01:
		if abs(t-t_d)<=0.01:
			n = 3	
		
	if n == 3:
		t = cur_time - time_init - n*t_d
		print(t)
		qr_d = np.array([traj3_30(t), traj3_31(t), traj3_32(t), traj3_33(t), traj3_34(t), traj3_35(t), traj3_36(t)])
		qdotr_d = np.array([traj3_dot_30(t), traj3_dot_31(t), traj3_dot_32(t), traj3_dot_33(t), traj3_dot_34(t), traj3_dot_35(t), traj3_dot_36(t)])
		qddotr_d = np.array([traj3_2dot_30(t), traj3_2dot_31(t), traj3_2dot_32(t), traj3_2dot_33(t), traj3_2dot_34(t), traj3_2dot_35(t), traj3_2dot_36(t)])
		
		qr = np.array(np.vectorize(bax.rarm.joint_angles().get)(rarm_mapping))
		qdotr = np.array(np.vectorize(bax.rarm.joint_velocities().get)(rarm_mapping))		
		
		psir = bax_controller.get_torque(qr, qdotr, qddotr_d, qr_d, qdotr_d, k1r, k2r, Ber)	
		R = dict(zip(rarm_mapping, (psir)))
		bax.rarm.set_joint_torques(R)

		#~ print(abs(qr-pos4))
		#~ print((abs(qr-pos4)<=0.3).all())
		#~ if ((abs(qr-pos4)<=0.05).all()) or abs(t-t_d)<=0.01:
		if abs(t-t_d)<=0.01:
			#~ n = 4	
			break		
						
	if n == 4:
		qr_d = np.array([0.75,0.8,3.05,0.5,0, 0, 0])
		bax.rarm.move_to_joint_positions(dict(zip(rarm_mapping, qr_d)))	
		thr = np.array(np.vectorize(bax.rarm.joint_angles().get)(rarm_mapping))
		if ((abs(thr-qr_d)<=0.05).all) or t >10:
			break

		#~ for i in range(len(pos1)):
			#~ (traj1[i],traj1_dot[i],traj1_2dot[i]) = trajectory.get_1trajectory(thr_init[i],pos1[i],3)
			#~ print(i)
		#~ qr_d = np.array([traj1[0], traj1[1], traj1[2], traj1[3], traj1[4], traj1[5], traj1[6]])
		#~ qdotr_d = np.array([traj1_dot[0], traj1_dot[1], traj1_dot[2], traj1_dot[3], traj1_dot[4], traj1_dot[5], traj1_dot[6]])
		#~ qddotr_d = np.array([traj1_2dot[0], traj1_2dot[1], traj1_2dot[2], traj1_2dot[3], traj1_2dot[4], traj1_2dot[5], traj1_2dot[6]])		
		
		

