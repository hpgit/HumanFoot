import numpy as np
from numpy.linalg import inv
from PyCommon.modules import pydart2 as pydart


class PDController:
    """
    :type h : float
    :type skel : pydart.Skeleton
    :type qhat : np.array
    """
    def __init__(self, skel, h):
        self.h = h
        self.skel = skel
        ndofs = self.skel.ndofs
        self.qhat = self.skel.q
        self.Kp = np.diagflat([0.0] * 6 + [400.0] * (ndofs - 6))
        self.Kd = np.diagflat([0.0] * 6 + [40.0] * (ndofs - 6))
        self.preoffset = 0.0

    def compute(self):
        skel = self.skel
        # deltaq =

        invM = inv(skel.M + self.Kd * self.h)
        p = -self.Kp.dot(skel.q + skel.dq * self.h - self.qhat)
        d = -self.Kd.dot(skel.dq)
        qddot = invM.dot(-skel.c + p + d + skel.constraint_forces())
        tau = p + d - self.Kd.dot(qddot) * self.h

        '''
        # Check the balance
        COP = skel.body('h_heel_left').to_world([0.05, 0, 0])
        offset = skel.C[0] - COP[0]

        # Adjust the target pose
        k1 = 200.0 if 0.0 < offset and offset < 0.1 else 2000.0
        k2 = 100.0
        kd = 10.0 if 0.0 < offset and offset < 0.1 else 100.0
        q_delta1 = np.array([-k1, -k2, -k1, -k2]) * offset
        q_delta2 = np.ones(4) * kd * (self.preoffset - offset)
        tau[np.array([17, 25, 19, 26])] += (q_delta1 + q_delta2)
        self.preoffset = offset
        #'''

        # Make sure the first six are zero
        tau[:6] = 0
        return tau

    def setTartgetPose(self, Rs):
        self.Rs = Rs

    def calcDeltaq(self):
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

        kt = Kt
        dt = Dt

        if weightMap is not None:
            kt = Kt * weightMap[0]
            dt = Dt * (weightMap[0]**.5)
            # dt = 0.
        a_des0 = kt*(p_r0 - p0) + dt*(v_r0 - v0) + a_r0
        ddth_des0 = kt*(mm.logSO3(np.dot(th0.transpose(), th_r0))) + dt*(dth_r0 - dth0) + ddth_r0
        # a_des0 = kt*(p_r0 - p0) + dt*(- v0) #+ a_r0
        # ddth_des0 = kt*(mm.logSO3(np.dot(th0.transpose(), th_r0))) + dt*(- dth0) #+ ddth_r0
        ddth_des[0] = np.concatenate((a_des0, ddth_des0))

        return deltaq
