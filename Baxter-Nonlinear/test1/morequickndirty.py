import rospy, baxter_interface, time, sys, os, math
import numpy as np
from std_msgs.msg import Empty, UInt16
from baxter_core_msgs.msg import SEAJointState
from math import pi, cos, sin
import random, threading

import bax_controller

D2R=pi/180.0;
R2D=1/D2R;

#~ def torque_controller(qd,qd_dot):

np.seterr(over='raise')
np.set_printoptions(precision=4)


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

rospy.init_node("qnda")
time.sleep(0.1)

pr = rospy.Publisher('robot/joint_state_publish_rate', UInt16, queue_size=1)
pr.publish(100)
pcdl = rospy.Publisher('robot/limb/left/suppress_cuff_interaction', Empty, queue_size=1)
pcdr = rospy.Publisher('robot/limb/right/suppress_cuff_interaction', Empty, queue_size=1)
gdl = rospy.Publisher('/robot/limb/left/suppress_gravity_compensation', Empty, queue_size=1)
gdr = rospy.Publisher('/robot/limb/right/suppress_gravity_compensation', Empty, queue_size=1)
gravityL = rospy.Subscriber('/robot/limb/left/gravity_compensation_torques', SEAJointState,callbackL)
gravityR = rospy.Subscriber('/robot/limb/right/gravity_compensation_torques', SEAJointState,callbackR)
r = rospy.Rate(100)

bax = Robot()

# admittance ICs
q_al = np.zeros(7)
qdot_al = np.zeros(7)
qddot_al = np.zeros(7)

q_ar = np.zeros(7)
qdot_ar = np.zeros(7)
qddot_ar = np.zeros(7)

tau_dl = np.zeros(7)
tau_dr = np.zeros(7)
#tau_d = np.array([-0.472, 0, -0.765, 0, 0, 0, -0.206])
## [0, -0.206, 0, 0, -0.472, 0, -0.765]
## [0, -0.139, 0, -0.020, 0, -0.153, 0]

# constants

M_d = 3
B_d = 1
K_d = 1
## 30000, 100, 300

# it works decently with just this PID
k_1 = 7 #35
beta = 9 #0.06

# use this to deactivate admittance control
adm_off = 0

#~ larm_mapping = np.array(["left_w0", "left_w1", "left_w2", "left_e0", "left_e1", "left_s0", "left_s1"])

larm_mapping = np.array(["left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"])
rarm_mapping = np.array(["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"])
##larm_reverse_mapping = dict((v,k) for k,v in larm_mapping.iteritems())

max_torques = np.array([50, 50, 50, 50, 15, 15, 15])
motor_scaling = np.array([0.7, 1, 0.7, 1, 0.4, 0.3, 0.015])
## [1.4, 1, 0.5, 2.2, 3.1, 2, 3.3]
## [0.001, 0.05, 0.005, 0.08, 2, 0.08, 1]

# create PID transform matrix
# not needed if we use the U = k_1*psi method
#PID = np.tile([25, 0, 2], (1, 7))
#np.multiply(np.diag([1.4,1,0.5,2.2,3.1,2,3.3]), PID, out=PID)

# more ICs
ql = np.array(np.vectorize(bax.larm.joint_angles().get)(larm_mapping))
qdotl = np.array(np.vectorize(bax.larm.joint_velocities().get)(larm_mapping))
qddotl = np.zeros(7)

# data logging preallocation
# right now the log_vars is nominal, the actual var selection is in the logging code
log_vars = ['t', 'q', 'qdot', 'q_a', 'qdot_a', 'qddot_a', 'q_d', 'qdot_d', 'tau_d', 'tau_app']
i_cap_time = 60
cap_rate = 60

log = np.zeros(shape=(int(i_cap_time*cap_rate), len(log_vars), 7))

##bax.larm.move_to_joint_positions(dict(zip(larm_mapping, q_d)))
time.sleep(0.2)
tau_app_l = np.array(np.vectorize(bax.larm.joint_efforts().get)(larm_mapping))
tau_app_r = np.array(np.vectorize(bax.rarm.joint_efforts().get)(rarm_mapping))
time.sleep(0.2)

def sd():
	ql_d = np.array([-0.75,0,0,0,0, 0, 0])
	qr_d = np.array([0.75,0,0,0,0, 0, 0])
	qdot_d = np.zeros(7)
	qddot_d = np.zeros(7)
	T = threading.Thread(target=bax.larm.move_to_joint_positions, args=(dict(zip(larm_mapping, ql_d)),))
	T.start()
	bax.rarm.move_to_joint_positions(dict(zip(rarm_mapping, qr_d)))
	for _ in xrange(100):
		if not rospy.is_shutdown():
			bax.larm.exit_control_mode()
			bax.rarm.exit_control_mode()
			pr.publish(100)  # 100Hz default joint state rate
			r.sleep()

rospy.on_shutdown(sd)

sd()

log_i = 0
time_since_logged = 0

# Gain
#LEFT
k11l = 10; k12l = 10; k13l = 8; k14l = 15; k15l = 10; k16l = 15; k17l = 10;
k1l_array = np.array([k11l, k12l, k13l, k14l, k15l, k16l, k17l]);
k1l = np.diag(k1l_array);

k21l = 10; k22l = 10; k23l = 1; k24l = 2; k25l = 0.5; k26l = 1; k27l = 1;
k2l_array = np.array([k21l, k22l, k23l, k24l, k25l, k26l, k27l]);
k2l = np.diag(k2l_array);

Be1l = 1;  Be2l = 1; Be3l = 1;  Be4l = 1; Be5l = 1; Be6l = 1; Be7l = 1;
Bel_array = np.array([Be1l, Be2l, Be3l, Be4l, Be5l, Be6l, Be7l]);
Bel = np.diag(Bel_array);

#RIGHT
k11r = 10; k12r = 10; k13r = 8; k14r = 15; k15r = 10; k16r = 15; k17r = 10;
k1r_array = np.array([k11r, k12r, k13r, k14r, k15r, k16r, k17r]);
k1r = np.diag(k1r_array);

k21r = 10; k22r = 8; k23r = 1; k24r = 2; k25r = 0.5; k26r = 1; k27r = 1;
k2r_array = np.array([k21r, k22r, k23r, k24r, k25r, k26r, k27r]);
k2r = np.diag(k2r_array);

Be1r = 1;  Be2r = 1; Be3r = 1;  Be4r = 1; Be5r = 1; Be6r = 1; Be7r = 1;
Ber_array = np.array([Be1r, Be2r, Be3r, Be4r, Be5r, Be6r, Be7r]);
Ber = np.diag(Ber_array);



time_init = rospy.get_time()
prev_time = time_init
n=1;
try:
	while not rospy.is_shutdown():

		pcdl.publish()
		pcdr.publish()
		gdl.publish()
		gdr.publish()
		#print gCompR
		#~ print('LEFT:')
		#~ print gCompL

		cur_time = rospy.get_time()
		dt = cur_time - prev_time
		prev_time = cur_time
		time_since_logged += dt
		t = cur_time - time_init
		#print(t)
		#print(sin(t))
		#print(cos(t))

		last_q_al = q_al
		last_qdot_al = qdot_al
		last_qddot_al = qddot_al

		last_q_ar = q_ar
		last_qdot_ar = qdot_ar
		last_qddot_ar = qddot_ar

		last_tau_app_l = tau_app_l
		last_tau_app_r = tau_app_r

		# read actual position and velocity from arm
		ql = np.array(np.vectorize(bax.larm.joint_angles().get)(larm_mapping))
		qdotl = np.array(np.vectorize(bax.larm.joint_velocities().get)(larm_mapping))
		#qddotl = last_qddot * 0.9 + 0.1 * qdot // float(dt)
		tau_extl = np.array(np.vectorize(bax.larm.joint_efforts().get)(larm_mapping))
		#qddot = np.zeros(7)

		qr = np.array(np.vectorize(bax.rarm.joint_angles().get)(rarm_mapping))
		qdotr = np.array(np.vectorize(bax.rarm.joint_velocities().get)(rarm_mapping))
		tau_extr = np.array(np.vectorize(bax.rarm.joint_efforts().get)(rarm_mapping))

		# need to do smoothed differentiation to get qddot

		#Q = np.array([q], [qdot], [np.zeros(7)])

		# tau_d - tau = M_d*qddot_a + B_d*qdot_a
		# need to solve this ^ for below values
		'''print tau_d
		print tau_app
		print last_qdot_a
		print last_q_a'''
		#print tau_app
		if not adm_off:
			#print "THINGS"
			#print tau_dl
			#print tau_app_l
			#print last_qdot_al
			#print last_q_al
			#print(tau_dl - tau_app_l)
			#print(last_qdot_al)
			#print B_d

			qddot_al = (tau_extl - tau_app_l - (last_qdot_al*B_d) - (last_q_al*K_d)) / (M_d)
			#print "CULPRIT"
			#print qddot_al
			qdot_al = last_qdot_al + qddot_al * dt
			q_al = last_q_al + qdot_al * dt

			qddot_ar = (tau_extr - tau_app_r - B_d*last_qdot_ar - K_d*last_q_ar) / M_d
			qdot_ar = last_qdot_ar + qddot_ar * dt
			q_ar = last_q_ar + qdot_ar * dt


		#Q_a = np.array([[q_a], [qdot_a], [qddot_a]])

		# everything is in joint space for now. eventually add kinematics etc to go
		# to cartesian

		# error calculation
		# xi = (q_d + q_a - q) * motor_scaling
		# xidot = (qdot_d + qdot_a - qdot) * motor_scaling
		# psi = np.clip((beta*xi+xidot)*k_1, -max_torques, max_torques)

		# PID control
		# print psi
		# if not np.isfinite(psi).any():
		#     sys.exit(1)
		# if np.greater(psi, 1000).any():
		#     print "BELUGA"
		#     sys.exit(100)

		#U = dict(zip(larm_mapping, (psi)))
		#print '********** ' + str(dt) + ' *************'

		sine = False

		if n==0:
			pos = 0
			vel = 0
			acc = 0
			ql_d = np.array([pos, pos, pos, pos, pos, pos, pos])
			qdotl_d = np.array([vel, vel, vel, vel, vel, vel, vel])
			qddotl_d = np.array([acc, acc, acc, acc, acc, acc, acc])
			el = ql_d - ql;
			pos = 0
			vel = 0
			acc = 0
			qr_d = np.array([pos, pos, pos, pos, pos, pos, pos])
			qdotr_d = np.array([vel, vel, vel, vel, vel, vel, vel])
			qddotr_d = np.array([acc, acc, acc, acc, acc, acc, acc])
			er = qr_d - qr;

		if n==1:
			wl = 1;
			if sine:
				pos = 10*D2R*sin(t)
				vel = 10*D2R*cos(t)
				acc = -10*D2R*sin(t)
				ql_d = np.array([-0.75, 10*D2R*sin(t), 10*D2R*sin(t), 20*D2R*sin(t), 20*D2R*sin(t), 30*D2R*sin(t), 30*D2R*sin(t)])
				qdotl_d = np.array([0, 10*D2R*cos(t), 10*D2R*cos(t), 20*D2R*cos(t), 20*D2R*cos(t), 30*D2R*cos(t), 30*D2R*cos(t)])
				qddotl_d = np.array([0, -10*D2R*sin(t), -10*D2R*sin(t), -20*D2R*sin(t), -20*D2R*sin(t), -30*D2R*sin(t), -30*D2R*sin(t)])
			else:
				ql_d = np.array([-0.75, 0, 0,0,0,0,0])
				qdotl_d = np.zeros(7)
				qddotl_d = np.zeros(7)
			el = ql_d - ql;
			#~ if (abs(el)<=0.1).all():
				#~ n=n+1
			if sine:
				qr_d = np.array([0.75, -10*D2R*sin(t), -10*D2R*sin(t), -20*D2R*sin(t), -20*D2R*sin(t), -30*D2R*sin(t), -30*D2R*sin(t)])
				qdotr_d = np.array([0, -10*D2R*cos(t), -10*D2R*cos(t), -20*D2R*cos(t), -20*D2R*cos(t), -30*D2R*cos(t), -30*D2R*cos(t)])
				qddotr_d = np.array([0, 10*D2R*sin(t), 10*D2R*sin(t), 20*D2R*sin(t), 20*D2R*sin(t), 30*D2R*sin(t), 30*D2R*sin(t)])
			else:
				qr_d = np.array([0.75, 0, 0,0,0,0,0])
				qdotr_d = np.zeros(7)
				qddotr_d = np.zeros(7)
			er = qr_d - qr;
			print(10*D2R*sin(t))

		# test for gravity
		elif n==2:
			wl = 1;
			if sine:
				ql_d = np.array([10*D2R*sin(wl*t), 10*D2R*cos(wl*t), 0, 0, 0, 0, 0])
				qdotl_d = np.array([10*wl*D2R*cos(wl*t), -10*wl*D2R*sin(wl*t), 0, 0, 0 ,0 ,0])
				qddotl_d = np.array([-10*wl*wl*D2R*sin(wl*t), -10*wl*wl*D2R*cos(wl*t), 0, 0, 0, 0, 0])
			else:
				ql_d = np.array([-0.75, 0, 0,0,0,0,0])
				qdotl_d = np.zeros(7)
				qddotl_d = np.zeros(7)
			el = ql_d - ql;
			#~ qr_d = np.array([10*D2R*sin(t), 10*D2R*cos(t), 0, 0, 0, 0, 0])
			#~ qdotr_d = np.array([10*D2R*cos(t), -10*D2R*sin(t), 0, 0, 0 ,0 ,0])
			#~ qddotr_d = np.array([-10*D2R*sin(t), -10*D2R*cos(t), 0, 0, 0, 0, 0])
			#~ er = qr_d - qr;
			wr = 1;
			if sine:
				qr_d = np.array([10*D2R*sin(wr*t), 10*D2R*cos(wr*t), 0, 0, 0, 0, 0])
				qdotr_d = np.array([10*wr*D2R*cos(wr*t), -10*wr*D2R*sin(wr*t), 0, 0, 0 ,0 ,0])
				qddotr_d = np.array([-10*wr*wr*D2R*sin(wr*t), -10*wr*wr*D2R*cos(wr*t), 0, 0, 0, 0, 0])
			else:
				qr_d = np.array([0.75, 0, 0,0,0,0,0])
				qdotr_d = np.zeros(7)
				qddotr_d = np.zeros(7)
			er = qr_d - qr;
			#~ if (abs(el)<=0.1).all():
				#~ n=n-1

		#~ print(n)
		#print('LEFT:')
		#print(el)
		#print(abs(el)<=0.1)
		#print('RIGHT:')
		#print(er)
		#print(abs(er)<=0.1)
		#print((abs(el)<=0.1).all())

		#print ql_d
		#print q_al
		psil = bax_controller.get_torque(ql, qdotl, qddotl_d, ql_d+q_al, qdotl_d+qdot_al, k1l, k2l, Bel)
		#~ psil = bax_controller.get_torque(ql, qdotl, qddotl_d, ql_d, qdotl_d, k1l, k2l, Bel)
		tau_app_l = psil.T
		#print "left torque:"
		#print psil

		e = ql_d - ql;
		#print("e =");

		#print(abs(e)<=0.03)
		#print(abs(e.all())<=0.03)
		#print((abs(e)<=0.05).all())
		el_dot = qdotl_d - qdotl;
		#print("e_dot = ")
		#print(e_dot)
		#print("\n")
		#print(Be)
		#print(psi)
		#print psil.astype(float).tolist()[0]
		L = dict(zip(larm_mapping, psil.astype(float).tolist()[0]))
		#~ #print L
#~ 
		#~ #PROBLEM 1:
		bax.larm.set_joint_torques(L)

		#~ psir = bax_controller.get_torque(qr, qdotr, qddotr_d, qr_d+q_ar, qdotr_d+qdot_ar, k1r, k2r, Ber)
		#~ psir = bax_controller.get_torque(qr, qdotr, qddotr_d, qr_d, qdotr_d, k1r, k2r, Ber)
		#~ tau_app_r = psir.T

		#~ R = dict(zip(rarm_mapping, psir.astype(float).tolist()[0]))

		#PROBLEM2:
		#~ bax.rarm.set_joint_torques(R)

		#~ print('\nleft')
		#~ print(psil)
		#~ print('\nright')
		#~ print(psir)
		r.sleep()
		#print "ran"


		#log
		# if time_since_logged > (1 / cap_rate):
		# 	if log_i >= log.shape[0]:
		# 		log.resize(log.shape[0]*2, log.shape[1], log.shape[2])
		# 	time_since_logged = 0
		# 	log[log_i, :, :] = np.array([np.ones(7) * (cur_time - time_init), ql, qdotl, q_al, qdot_al, qddot_al, ql_d, qdotl_d, tau_dl, tau_app_l])
		# 	log_i += 1







except KeyboardInterrupt:
	sd()


#~ qd1 = np.array([0,0,0,0,0, -0.75, 0])
#~ qd_dot1 = np.zeros(7)
#~ qd2 = np.array([0,0,0,0,0, -0.45, 0.30])
#~ qd_dot2 = np.zeros(7)
#~ n=1;
#~ [q1, q_dot1, q_2dot1] = torque_controller(qd1,qd_dot1)
#~ print(norm(qd1-q1))
#~ if n==1:
	#~ [q1, q_dot1, q_2dot1] = torque_controller(qd1,qd_dot1)
	#~ if
	#~ n = n+1
#~ elif n==2:
	#~ torque_controller(qd2,qd_dot2)
	#~ n = n-1
