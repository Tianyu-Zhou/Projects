# -*- coding: utf-8 -*-

from sympy import *

def get_1trajectory(th1,th2,t):
    a10, a11, a12, a13 = symbols('a10 a11 a12 a13')
    a = solve([-th1 + a10,
              -th2 + a10 + a11*t + a12*t**2 + a13*t**3,
              a11,
              a11 + 2*a12*t + 3*a13*t**2],
              [a10,a11,a12,a13]);
    t = Symbol('t')
    traj1 = lambdify(t, a[a10] + a[a11]*t + a[a12]*t**2 + a[a13]*t**3);
    traj1_dot = lambdify(t, a[a11] + 2*a[a12]*t + 3*a[a13]*t**2);
    traj1_2dot = lambdify(t, 2*a[a12]*t + 6*a[a13]*t);
    return traj1, traj1_dot, traj1_2dot
    
def get_3trajectory(th1,th2,th3,th4,t):
    a10, a11, a12, a13 = symbols('a10 a11 a12 a13')
    a20, a21, a22, a23 = symbols('a20 a21 a22 a23')
    a30, a31, a32, a33 = symbols('a30 a31 a32 a33')
#    t = Symbol('t')
#    th1, th2, th3, th4 = symbols('th1 th2 th3 th4')
#    th1 = 0.2; th2 = 0.4; th3 = 0.6; th4 = 0.8; t = 3;
    
    a = solve([-th1 + a10,
              -th2 + a10 + a11*t + a12*t**2 + a13*t**3,
              -th2 + a20,
              -th3 + a20 + a21*t + a22*t**2 + a23*t**3,
              -th3 + a30,
              -th4 + a30 + a31*t + a32*t**2 + a33*t**3,
              a11,
              a11 + 2*a12*t + 3*a13*t**2 - a21,
              a21 + 2*a22*t + 3*a23*t**2 - a31,
              a31 + 2*a32*t + 3*a33*t**2,
              2*a12 + 6*a13*t - 2*a22,
              2*a22 + 6*a23*t - 2*a32],
              [a10,a11,a12,a13,a20,a21,a22,a23,a30,a31,a32,a33]);
    

    t = Symbol('t')
    traj1 = lambdify(t, a[a10] + a[a11]*t + a[a12]*t**2 + a[a13]*t**3);
    traj2 = lambdify(t, a[a20] + a[a21]*t + a[a22]*t**2 + a[a23]*t**3);
    traj3 = lambdify(t, a[a30] + a[a31]*t + a[a32]*t**2 + a[a33]*t**3);
    
    traj1_dot = lambdify(t, a[a11] + 2*a[a12]*t + 3*a[a13]*t**2);
    traj2_dot = lambdify(t, a[a21] + 2*a[a22]*t + 3*a[a23]*t**2);
    traj3_dot = lambdify(t, a[a31] + 2*a[a32]*t + 3*a[a33]*t**2);
    
    traj1_2dot = lambdify(t, 2*a[a12]*t + 6*a[a13]*t);
    traj2_2dot = lambdify(t, 2*a[a22]*t + 6*a[a23]*t);
    traj3_2dot = lambdify(t, 2*a[a32]*t + 6*a[a33]*t);
    return traj1, traj2, traj3, traj1_dot, traj2_dot, traj3_dot, traj1_2dot, traj2_2dot, traj3_2dot

if __name__ == "__main__":
    (traj1,traj1_dot,traj1_2dot) = get_1trajectory(0.4,0.8,3)
    print(traj1)
    print(traj1_dot)
    print('\n')
    (traj1,traj2,traj3,traj1_dot,traj2_dot,traj3_dot,traj1_2dot,traj2_2dot,traj3_2dot) = get_3trajectory(0.2,0.4,0.6,0.8,3)
    print(traj1)
    print(traj2)
    print(traj3)
