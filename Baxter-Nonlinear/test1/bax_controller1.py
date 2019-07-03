# -*- coding: utf-8 -*-

import numpy as np
from math import pi, cos, sin
import random

D2R=pi/180.0;
R2D=1/D2R;

#~ #Gain
#~ k1 = 1; k2 = 1;
#~
#~ q1=30*D2R; q2=45*D2R; q3=50*D2R; q4=67*D2R; q5=20*D2R; q6=33*D2R; q7=80*D2R;
#~
#~ q1_dot = 1; q2_dot = 1; q3_dot = 1; q4_dot = 1; q5_dot = 1; q6_dot = 1; q7_dot = 1;
#~ q1_2dot = 1; q2_2dot = 1; q3_2dot = 1; q4_2dot = 1; q5_2dot = 1; q6_2dot = 1; q7_2dot = 1;
#~
#~ qd1_dot = 2; qd2_dot = 2; qd3_dot = 2; qd4_dot = 2; qd5_dot = 2; qd6_dot = 2; qd7_dot = 2;
#~
#~
#~ # Current Baxter state
#~ q = np.array([q1, q2, q3, q4, q5, q6, q7]);
#~ q_dot = np.array([q1_dot, q2_dot, q3_dot, q4_dot, q5_dot, q6_dot, q7_dot]);
#~ q_2dot = np.array([q1_2dot, q2_2dot, q3_2dot, q4_2dot, q5_2dot, q6_2dot, q7_2dot]);
#~
#~ # Design Baxter state
#~ qd = q + 0.05;
#~ qd_dot = np.array([qd1_dot, qd2_dot, qd3_dot, qd4_dot, qd5_dot, qd6_dot, qd7_dot]);
#~
#~ # We need Joint positon q(th), Angular velocity q_dot, Acceleration q_2dot and Center of mass (x,y,z)
#~ # What we need to design are Gains k1 and k2, Angular velocity qd_dot and Acceleration qd_2dot
#~
#~ # get_torque(joint positon(angular), joint position (x,y,z), angular velocity, acceleration, design jiont position, design angular velocity, Gain)
#~
#~ Be1 = 1;  Be2 = 1; Be3 = 1;  Be4 = 1; Be5 = 1;  Be6 = 1; Be7 = 1;
#~ Be_array = np.array([Be1, Be2, Be3, Be4, Be5, Be6, Be7]);
#~ Be = np.diag(Be_array);
#~ print(Be)

def get_torque(q,q_dot,q_2dot,qd,qd_dot,k1,k2,Be):
    # Error
    th = q;
    #th[1] = q[1] + pi/2;
    e = qd - q;
    e_dot = qd_dot - q_dot;
    # print "guilty"
    # print e
    # print e_dot
    # print k1
    r = e_dot + k1.dot(e);

    # =============================================================================
    # Fixed parameter
    # =============================================================================
    g = np.array([0, 0, -9.81, 0]);

    a1=0.069; a2=0; a3=0.069; a4=0;	a5=0.01; a6=0; a7=0;

    S1=0.2703; S2=0; S3=0.3644;	S4=0; S5=0.3743; S6=0; S7=0.2295;

    al01=-pi/2; al12=pi/2; al23=-pi/2; al34=pi/2; al45=-pi/2; al56=pi/2; al67=0;

    # Center of mass
    x1 = -0.05117; y1 = 0.07908;   z1 = 0.00086;
    x2 = 0.00269;  y2 = -0.00529;  z2 = 0.06845;
    x3 = -0.07176; y3 = 0.08149;   z3 = 0.00132;
    x4 = 0.00159;  y4 = -0.01117;  z4 = 0.02618;
    x5 = -0.01168; y5 = 0.13111;   z5 = 0.0046;
    x6 = 0.00697;  y6 = 0.006;     z6 = 0.06048;
    x7 = 0.005137; y7 = 0.0009572; z7 = -0.06682;

    r1_vec = np.array([[x1], [y1], [z1], [1]]);
    r2_vec = np.array([[x2], [y2], [z2], [1]]);
    r3_vec = np.array([[x3], [y3], [z3], [1]]);
    r4_vec = np.array([[x4], [y4], [z4], [1]]);
    r5_vec = np.array([[x5], [y5], [z5], [1]]);
    r6_vec = np.array([[x6], [y6], [z6], [1]]);
    r7_vec = np.array([[x7], [y7], [z7], [1]]);
    r_vec = np.array([r1_vec, r2_vec, r3_vec, r4_vec, r5_vec, r6_vec, r7_vec])

    Ixx1 = 0.0470910226; Iyy1 = 0.035959884;  Izz1 = 0.0376697645;
    Ixx2 = 0.027885975;  Iyy2 = 0.020787492;  Izz2 = 0.0117520941;
    Ixx3 = 0.0266173355; Iyy3 = 0.012480083;  Izz3 = 0.0284435520;
    Ixx4 = 0.0131822787; Iyy4 = 0.009268520;  Izz4 = 0.0071158268;
    Ixx5 = 0.0166774282; Iyy5 = 0.003746311;  Izz5 = 0.0167545726;
    Ixx6 = 0.0070053791; Iyy6 = 0.005527552;  Izz6 = 0.0038760715;
    Ixx7 = 0.0008162135; Iyy7 = 0.0008735012; Izz7 = 0.0005494148;

    Ixy1 = -0.0061487003; Iyz1 = -0.0007808689; Ixz1 = 0.0001278755;
    Ixy2 = -0.0001882199; Iyz2 = 0.0020767576;  Ixz2 = -0.00030096397;
    Ixy3 = -0.0039218988; Iyz3 = -0.001083893;  Ixz3 = 0.0002927063;
    Ixy4 = -0.0001966341; Iyz4 = 0.000745949;   Ixz4 = 0.0003603617;
    Ixy5 = -0.0001865762; Iyz5 = 0.0006473235;  Ixz5 = 0.0001840370;
    Ixy6 = 0.0001534806;  Iyz6 = -0.0002111503; Ixz6 = -0.0004438478;
    Ixy7 = 0.000128440;   Iyz7 = 0.0001057726;  Ixz7 = 0.00018969891;

    m1=5.70044; m2=3.22698; m3=4.31272; m4=2.07206; m5=2.24665; m6=1.60979; m7=0.54218;

    s01 =sin(al01); c01 =cos(al01); s12 =sin(al12); c12 =cos(al12); s23 =sin(al23); c23 =cos(al23);
    s34 =sin(al34); c34 =cos(al34); s45 =sin(al45); c45 =cos(al45); s56 =sin(al56); c56 =cos(al56);
    s67 =sin(al67); c67 =cos(al67);

    T00 = np.mat([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]], dtype=float);

    T11=T00; T22=T00; T33=T00; T44=T00; T55=T00; T66=T00;

    # Change parameter to array
    a = np.array([a1,a2,a3,a4,a5,a6,a7]);
    S = np.array([S1,S2,S3,S4,S5,S6,S7]);
    al = np.array([al01,al12,al23,al34,al45,al56,al67]);

    Ixx = np.array([Ixx1, Ixx2, Ixx3, Ixx4, Ixx5, Ixx6, Ixx7]);
    Iyy = np.array([Iyy1, Iyy2, Iyy3, Iyy4, Iyy5, Iyy6, Iyy7]);
    Izz = np.array([Izz1, Izz2, Izz3, Izz4, Izz5, Izz6, Izz7]);
    Ixy = np.array([Ixy1, Ixy2, Ixy3, Ixy4, Ixy5, Ixy6, Ixy7]);
    Iyz = np.array([Iyz1, Iyz2, Iyz3, Iyz4, Iyz5, Iyz6, Iyz7]);
    Ixz = np.array([Ixz1, Ixz2, Ixz3, Ixz4, Ixz5, Ixz6, Ixz7]);
    m = np.array([m1, m2, m3, m4, m5, m6, m7]);
    x = np.array([x1,x2,x3,x4,x5,x6,x7]);
    y = np.array([y1,y2,y3,y4,y5,y6,y7]);
    z = np.array([z1,z2,z3,z4,z5,z6,z7]);

    #Transform matrix
    T = np.zeros((8,8), dtype=object);
    for i in range(1,8):
        T[i-1][i] = np.mat([[cos(th[i-1]), -cos(al[i-1])*sin(th[i-1]), sin(al[i-1])*sin(th[i-1]), a[i-1]*cos(th[i-1])],
                              [sin(th[i-1]), cos(al[i-1])*cos(th[i-1]), -sin(al[i-1])*cos(th[i-1]), a[i-1]*sin(th[i-1])],
                              [0, sin(al[i-1]), cos(al[i-1]), S[i-1]],
                              [0, 0, 0, 1]]);

    T[0][0] = T00; T[1][1] = T11; T[2][2] = T22; T[3][3] = T33; T[4][4] = T44; T[5][5] = T55; T[6][6] = T66;

    T[0][2] = T[0][1]*T[1][2];
    T[0][3] = T[0][1]*T[1][2]*T[2][3];
    T[0][4] = T[0][1]*T[1][2]*T[2][3]*T[3][4];
    T[0][5] = T[0][1]*T[1][2]*T[2][3]*T[3][4]*T[4][5];
    T[0][6] = T[0][1]*T[1][2]*T[2][3]*T[3][4]*T[4][5]*T[5][6];
    T[0][7] = T[0][1]*T[1][2]*T[2][3]*T[3][4]*T[4][5]*T[5][6]*T[6][7];

    T[1][3] = T[1][2]*T[2][3];
    T[1][4] = T[1][2]*T[2][3]*T[3][4];
    T[1][5] = T[1][2]*T[2][3]*T[3][4]*T[4][5];
    T[1][6] = T[1][2]*T[2][3]*T[3][4]*T[4][5]*T[5][6];
    T[1][7] = T[1][2]*T[2][3]*T[3][4]*T[4][5]*T[5][6]*T[6][7];

    T[2][4] = T[2][3]*T[3][4];
    T[2][5] = T[2][3]*T[3][4]*T[4][5];
    T[2][6] = T[2][3]*T[3][4]*T[4][5]*T[5][6];
    T[2][7] = T[2][3]*T[3][4]*T[4][5]*T[5][6]*T[6][7];

    T[3][5] = T[3][4]*T[4][5];
    T[3][6] = T[3][4]*T[4][5]*T[5][6];
    T[3][7] = T[3][4]*T[4][5]*T[5][6]*T[6][7];

    T[4][6] = T[4][5]*T[5][6];
    T[4][7] = T[4][5]*T[5][6]*T[6][7];

    T[5][7] = T[5][6]*T[6][7];

    # Jacobian matrix
    J = np.zeros(7, dtype=object);

    for i in range(7):
        J[i] = np.mat([[(-Ixx[i]+Iyy[i]+Izz[i])/2, Ixy[i], Ixz[i], m[i]*x[i]],
                      [Ixy[i], (Ixx[i]-Iyy[i]+Izz[i])/2, Iyz[i], m[i]*y[i]],
                      [Ixz[i], Iyz[i], (Ixx[i]+Iyy[i]-Izz[i])/2, m[i]*z[i]],
                      [m[i]*x[i], m[i]*y[i], m[i]*z[i], m[i]]]);

    Q = np.mat([[0, -1, 0, 0],
                [1, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0]], dtype=float);

    # Get Uij matrix
    U2 = np.zeros((7,7), dtype=object);

    for i in range(1,8):
        for j in range(1,8):
            if j<=i:
                U2[i-1][j-1] = T[0][j-1]*Q*T[j-1][i];
            else:
                U2[i-1][j-1] = 0;

    # M matrix
    M = np.zeros((7,7))
    for i in range(7):
		for k in range(7):
			if i>=k:
				for j in range(i,7):
					M[i][k] = M[i][k] + np.trace(U2[j][k]*J[j]*U2[j][i].T);
			else:
				for j in range(k,7):
					M[i][k] = M[i][k] + np.trace(U2[j][k]*J[j]*U2[j][i].T);

    # G matrix
    G = np.zeros((7,1));
    for i in range(7):
		for j in range(i,7):
				G[i] = G[i] + -m[j]*g*U2[j][i]*r_vec[j];

    # Get Uijk matrix
    U3 = np.zeros((7,7,7),dtype=object);

    for i in range(1,8):
        for j in range(1,8):
            for k in range(1,8):
                if i>=k and k>=j:
                    U3[i-1][j-1][k-1] = T[0][j-1]*Q*T[j-1][k-1]*Q*T[k-1][i];
                elif i>=j and j>=k:
                    U3[i-1][j-1][k-1] = T[0][k-1]*Q*T[k-1][j-1]*Q*T[j-1][i];
                else:
                    U3[i-1][j-1][k-1] = 0;

    # h = np.zeros((7,7,7),dtype=object);
    #
    # for i in range(7):
    #     for j in range(7):
    #         for k in range(7):
    #             h[i][j][k] = np.trace(U3[i][j][k]*J[i]*U2[i][i].T)
    h = np.zeros((7,7,7),dtype=object);
    for i in range(7):
		for k in range(7):
			for m in range(7):
				if i>=k and i>=m:
					for j in range(i,7):
						h[i][k][m] = h[i][k][m] + np.trace(U3[j][k][m]*J[j]*U2[j][i].T)
				elif k>=m and k>=i:
					for j in range(k,7):
						h[i][k][m] = h[i][k][m] + np.trace(U3[j][k][m]*J[j]*U2[j][i].T)
				elif m>=i and m>=k:
					for j in range(m,7):
						h[i][k][m] = h[i][k][m] + np.trace(U3[j][k][m]*J[j]*U2[j][i].T)

    # =============================================================================
    # C = np.zeros((7,1))
    #
    # for i in range(7):
    #     for j in range(7):
    #         for k in range(7):
    #             C[i][0] = C[i][0] + h[i][j][k]*q_dot[j]*q_dot[k]
    # =============================================================================

    # V matrix
    V = h.dot(q_dot);
    #print M.dot(q_2dot)
    # print "START"
    # print Be
    # print M
    # print q_2dot
    # print V
    # print qd_dot
    # print k1
    # print "G:"
    # print G
    # print r
    # print e
    # print k2
    t1 = Be.dot(M.dot(q_2dot.T))
    t2 = V.dot(qd_dot.T + k1.dot(e))
    t3 = G.T
    t4 = (k1).dot(M).dot(r.T-k1.dot(e.T))
    t5 = e.T
    t6 = k2.dot(r.T)
    # print "terms:"
    # print t1
    # print t2
    # print t3
    # print t4
    # print t5
    # print t6
    torque =  t1 + t2 + t3*0 + t4 + t5 + t6  #- np.reshape(tau_ext,(7,1));
    # print "torque:"
    # print torque
    return torque

if __name__ == "__main__":
    torque = get_torque(q,q_dot,q_2dot,qd,qd_dot,k1,k2,Be);
    print(torque)
