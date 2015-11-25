import numpy as np

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mm

# th_r[0], th[0] : (Vec3, SO3), th_r[1:], th[1:] : SO3
# dth_r, dth : Vec3
# ddth_r : Vec3
def getDesiredDOFAccelerations(th_r, th, dth_r, dth, ddth_r, Kt, Dt, weightMap=None):
    ddth_des = [None]*len(th_r)
    
#    p_r0 = mm.T2p(th_r[0])
#    p0 = mm.T2p(th[0])
#    v_r0 = dth_r[0][0:3]
#    v0 = dth[0][0:3]
#    a_r0 = ddth_r[0][0:3]
#    th_r0 = mm.T2R(th_r[0])
#    th0 = mm.T2R(th[0])
#    dth_r0 = dth_r[0][3:6]
#    dth0 = dth[0][3:6]
#    ddth_r0 = ddth_r[0][3:6]

    p_r0 = th_r[0][0]
    p0 = th[0][0]
    v_r0 = dth_r[0][0:3]
    v0 = dth[0][0:3]
    a_r0 = ddth_r[0][0:3]
    
    th_r0 = th_r[0][1]
    th0 = th[0][1]
    dth_r0 = dth_r[0][3:6]
    dth0 = dth[0][3:6]
    ddth_r0 = ddth_r[0][3:6]

    kt = 1.
    dt = 1.
    print weightMap
    if weightMap is not None:
        kt = weightMap[0]
        dt = 2*(kt**.5)
    a_des0 = Kt*kt*(p_r0 - p0) + Dt*dt*(v_r0 - v0) #+ a_r0
    ddth_des0 = Kt*kt*(mm.logSO3(np.dot(th0.transpose(), th_r0))) + Dt*dt*(dth_r0 - dth0) #+ ddth_r0
    ddth_des[0] = np.concatenate((a_des0, ddth_des0))
    
    for i in range(1, len(th_r)):
        if weightMap is not None:
            kt = weightMap[0]
            dt = 2*(kt**.5)
        ddth_des[i] = Kt*kt*(mm.logSO3(np.dot(th[i].transpose(), th_r[i]))) + Dt*dt*(dth_r[i] - dth[i]) #+ ddth_r[i]

    return ddth_des

# th_r, th : SO3
# dth_r, dth : Vec3
# ddth_r : Vec3
def getDesiredAngAccelerations(th_r, th, dth_r, dth, ddth_r, Kt, Dt):
    ddth_des = [None]*len(th_r) 
    for i in range(len(th_r)):
        ddth_des[i] = Kt*(mm.logSO3(np.dot(th[i].transpose(), th_r[i]))) + Dt*(dth_r[i] - dth[i]) + ddth_r[i]

    return ddth_des

# p_r, p, v_r, v, a_r : Vec3
def getDesiredAcceleration(p_r, p, v_r, v, a_r, Kt, Dt):
    return Kt*(p_r - p) + Dt*(v_r - v) + a_r

def getIntegralDOF(th, d_th, dt):
    new_th = [None]*len(th)
    v_r0 = d_th[0][0:3]
    v_r1 = d_th[0][3:6]
    new_th0_l = th[0][0] + [v_r0[0]*dt, v_r0[1]*dt, v_r0[2]*dt]
    new_th0_a = np.dot(th[0][1], mm.exp([v_r1[0]*dt, v_r1[1]*dt, v_r1[2]*dt]))
    new_th[0] = [None]*2
    new_th[0][0] = new_th0_l
    new_th[0][1] = new_th0_a
    for i in range(1, len(th)):
        new_th[i] = np.dot(th[i], mm.exp([d_th[i][0]*dt, d_th[i][1]*dt, d_th[i][2]*dt]))
    return new_th
